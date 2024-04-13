#!/usr/bin/env python

import rospy
import pymongo as pm

class ClearMessageStore(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name', 'message_store')
        self.collection_name = rospy.get_param('~collection_name', 'message_store')
        self.db_port = rospy.get_param('~db_port', 27018)
        self.__clear_message_store()

    def __clear_message_store(self):
        try:
            client = pm.MongoClient(port=self.db_port)
            database = client[self.db_name]
            collection = database[self.collection_name]
            collection.delete_many({})
        except Exception as exc:
            rospy.logerr(str(exc))

if __name__ == '__main__':
    rospy.init_node('clear_message_store')
    try:
        ClearMessageStore()
    except rospy.ROSInterruptException:
        pass
