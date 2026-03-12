#!/usr/bin/env python3
"""
FastReID Person Recognition Server

Runs on the slave laptop (RTX 3070). Uses torchreid/OSNet to extract
appearance feature embeddings and match people by cosine similarity.

Replaces VLM-based person recognition — same /vlm/query service interface,
only handles: save_face, recognize_person, clear_faces.

Laptop Setup:
    pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
    pip3 install torchreid opencv-python numpy
    OR:
    pip3 install boxmot  (includes reid models)

Set ROS env:
    export ROS_MASTER_URI=http://192.168.50.44:11311
    export ROS_IP=<laptop_ip>

Run:
    rosrun hsr_task_sm reid_server.py
    # Or with custom model:
    rosrun hsr_task_sm reid_server.py _model:=osnet_x1_0
"""

import base64
import os
import numpy as np
import rospy

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

from hsr_task_sm.srv import VLMQuery, VLMQueryResponse

FACES_DIR = os.path.expanduser('~/.hsr_reid_features')
DEVICE = 'cuda' if (TORCH_AVAILABLE and torch.cuda.is_available()) else 'cpu'

# Person crop size expected by OSNet
CROP_SIZE = (256, 128)  # (height, width)


class ReIDServer:

    def __init__(self):
        rospy.init_node('reid_server', anonymous=False)

        self.model_name = rospy.get_param('~model', 'osnet_x1_0')
        self.threshold = rospy.get_param('~threshold', 0.5)  # cosine similarity threshold

        os.makedirs(FACES_DIR, exist_ok=True)

        rospy.loginfo('[ReIDServer] Device: %s', DEVICE)
        rospy.loginfo('[ReIDServer] Features stored in: %s', FACES_DIR)

        self.extractor = None
        self._load_model()

        # In-memory feature cache: {name: np.array}
        self._features = {}
        self._load_saved_features()

        self.service = rospy.Service('/reid/query', VLMQuery, self._handle_query)
        rospy.loginfo('[ReIDServer] Ready on /reid/query')

    # ------------------------------------------------------------------

    def _load_model(self):
        """Load OSNet feature extractor."""
        if not TORCH_AVAILABLE or not CV_AVAILABLE:
            rospy.logerr('[ReIDServer] torch or cv2 not installed')
            return
        try:
            import torchreid
            self.extractor = torchreid.utils.FeatureExtractor(
                model_name=self.model_name,
                model_path='',  # auto-download pretrained weights
                device=DEVICE,
            )
            rospy.loginfo('[ReIDServer] Loaded model: %s on %s', self.model_name, DEVICE)
        except Exception as e:
            rospy.logerr('[ReIDServer] Failed to load torchreid model: %s', e)
            # Fallback: try torchvision ResNet
            try:
                self._load_torchvision_fallback()
            except Exception as e2:
                rospy.logerr('[ReIDServer] Fallback also failed: %s', e2)

    def _load_torchvision_fallback(self):
        """Use ResNet50 from torchvision as fallback feature extractor."""
        import torchvision.models as models
        import torchvision.transforms as T

        model = models.resnet50(pretrained=True)
        # Remove final FC layer — use pool5 features (2048-dim)
        model = torch.nn.Sequential(*list(model.children())[:-1])
        model.eval().to(DEVICE)

        self._transform = T.Compose([
            T.ToPILImage(),
            T.Resize(CROP_SIZE),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        self._fallback_model = model
        self.extractor = None  # signal to use fallback path
        rospy.logwarn('[ReIDServer] Using torchvision ResNet50 fallback')

    # ------------------------------------------------------------------

    def _extract_features(self, img_bgr):
        """
        Extract a normalized appearance feature vector from a BGR image.
        Returns np.array of shape (D,) or None on failure.
        """
        if img_bgr is None or img_bgr.size == 0:
            return None

        # Crop/resize to ReID input size
        person_img = cv2.resize(img_bgr, (CROP_SIZE[1], CROP_SIZE[0]))

        try:
            if self.extractor is not None:
                # torchreid path
                feat = self.extractor([person_img])  # returns tensor (1, D)
                feat = feat.cpu().numpy()[0]
            else:
                # torchvision fallback
                img_rgb = cv2.cvtColor(person_img, cv2.COLOR_BGR2RGB)
                t = self._transform(img_rgb).unsqueeze(0).to(DEVICE)
                with torch.no_grad():
                    feat = self._fallback_model(t).squeeze().cpu().numpy()

            # L2 normalize
            feat = feat / (np.linalg.norm(feat) + 1e-8)
            return feat.astype(np.float32)
        except Exception as e:
            rospy.logerr('[ReIDServer] Feature extraction error: %s', e)
            return None

    def _cosine_similarity(self, a, b):
        return float(np.dot(a, b))

    # ------------------------------------------------------------------

    def _load_saved_features(self):
        """Load feature vectors from disk into memory cache."""
        self._features = {}
        for fname in os.listdir(FACES_DIR):
            if fname.endswith('.npy'):
                name = fname[:-4]
                path = os.path.join(FACES_DIR, fname)
                self._features[name] = np.load(path)
        rospy.loginfo('[ReIDServer] Loaded %d saved person features', len(self._features))

    def _save_feature(self, name, feat):
        path = os.path.join(FACES_DIR, f'{name}.npy')
        np.save(path, feat)
        self._features[name] = feat

    # ------------------------------------------------------------------

    def _handle_query(self, req):
        query_type = req.query_type.strip().lower()

        if query_type == 'save_face':
            return self._handle_save(req)
        elif query_type == 'recognize_person':
            return self._handle_recognize(req)
        elif query_type == 'clear_faces':
            return self._handle_clear()
        else:
            return VLMQueryResponse(
                answer='', success=False,
                reason=f'reid_server only handles: save_face, recognize_person, clear_faces. Got: {query_type}'
            )

    def _handle_save(self, req):
        name = (req.context or '').strip()
        if not name:
            return VLMQueryResponse(answer='failed', success=False,
                                    reason='context must be the person name')
        if not req.image_base64:
            return VLMQueryResponse(answer='failed', success=False,
                                    reason='image_base64 is empty')
        try:
            img_bytes = base64.b64decode(req.image_base64)
            img_arr = np.frombuffer(img_bytes, dtype=np.uint8)
            img_bgr = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

            feat = self._extract_features(img_bgr)
            if feat is None:
                return VLMQueryResponse(answer='failed', success=False,
                                        reason='Feature extraction failed')

            self._save_feature(name, feat)
            rospy.loginfo('[ReIDServer] Saved features for "%s" (dim=%d)', name, len(feat))
            return VLMQueryResponse(answer='saved', success=True,
                                    reason=f'Saved {len(feat)}-dim feature for {name}')
        except Exception as e:
            rospy.logerr('[ReIDServer] save_face error: %s', e)
            return VLMQueryResponse(answer='failed', success=False, reason=str(e))

    def _handle_recognize(self, req):
        if not req.image_base64:
            return VLMQueryResponse(answer='unknown', success=False,
                                    reason='image_base64 is empty')
        if not self._features:
            return VLMQueryResponse(answer='unknown', success=False,
                                    reason='No persons enrolled. Call save_face first.')
        try:
            img_bytes = base64.b64decode(req.image_base64)
            img_arr = np.frombuffer(img_bytes, dtype=np.uint8)
            img_bgr = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

            query_feat = self._extract_features(img_bgr)
            if query_feat is None:
                return VLMQueryResponse(answer='unknown', success=False,
                                        reason='Feature extraction failed on query image')

            # Cosine similarity against all saved features
            scores = {name: self._cosine_similarity(query_feat, feat)
                      for name, feat in self._features.items()}

            best_name = max(scores, key=scores.get)
            best_score = scores[best_name]

            rospy.loginfo('[ReIDServer] Scores: %s  -> best: %s (%.3f)',
                          {n: f'{s:.3f}' for n, s in scores.items()}, best_name, best_score)

            if best_score >= self.threshold:
                return VLMQueryResponse(answer=best_name, success=True,
                                        reason=f'score={best_score:.3f}')
            else:
                rospy.logwarn('[ReIDServer] Best score %.3f below threshold %.3f',
                              best_score, self.threshold)
                return VLMQueryResponse(answer='unknown', success=False,
                                        reason=f'Best match {best_name} score={best_score:.3f} below threshold={self.threshold}')
        except Exception as e:
            rospy.logerr('[ReIDServer] recognize error: %s', e)
            return VLMQueryResponse(answer='unknown', success=False, reason=str(e))

    def _handle_clear(self):
        try:
            count = 0
            for fname in os.listdir(FACES_DIR):
                if fname.endswith('.npy'):
                    os.remove(os.path.join(FACES_DIR, fname))
                    count += 1
            self._features = {}
            rospy.loginfo('[ReIDServer] Cleared %d person features', count)
            return VLMQueryResponse(answer='cleared', success=True,
                                    reason=f'Deleted {count} feature file(s)')
        except Exception as e:
            return VLMQueryResponse(answer='', success=False, reason=str(e))

    def run(self):
        rospy.spin()


def main():
    try:
        ReIDServer().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
