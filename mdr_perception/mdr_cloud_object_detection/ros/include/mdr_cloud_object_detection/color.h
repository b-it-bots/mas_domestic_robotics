/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sergey Alexandrov
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_COLOR_H
#define MDR_CLOUD_OBJECT_DETECTION_COLOR_H

#include <std_msgs/ColorRGBA.h>
#include <mdr_cloud_object_detection/aliases.h>

namespace mdr_cloud_object_detection
{

struct Color
{
    uint8_t mRed;
    uint8_t mGreen;
    uint8_t mBlue;

    Color(float pRed, float pGreen, float pBlue)
    : mRed(static_cast<uint8_t>(pRed * 255)), mGreen(static_cast<uint8_t>(pGreen * 255)),
      mBlue(static_cast<uint8_t>(pBlue * 255))
    { }

    Color(int pRed, int pGreen, int pBlue)
    : mRed(static_cast<uint8_t>(pRed)), mGreen(static_cast<uint8_t>(pGreen)), mBlue(static_cast<uint8_t>(pBlue)) { }

    enum Name
    {
        SALMON,
        TEAL,
        DEEP_PINK,
        SANGRIA,
        SEA_BLUE,
        SEA_GREEN,
        SCARLET,
        PUMPKIN,
        JASMINE,
        IVORY,
        GAINSBORO,
    };

    explicit Color(Name name)
    {
        switch (name)
        {
            case SALMON:
                mRed = 0xFA, mGreen = 0x80, mBlue = 0x72;
                break;
            case TEAL:
                mRed = 0x00, mGreen = 0x80, mBlue = 0x80;
                break;
            case DEEP_PINK:
                mRed = 0xFF, mGreen = 0x14, mBlue = 0x93;
                break;
            case SANGRIA:
                mRed = 0x92, mGreen = 0x00, mBlue = 0x0A;
                break;
            case SEA_BLUE:
                mRed = 0x00, mGreen = 0x69, mBlue = 0x94;
                break;
            case SEA_GREEN:
                mRed = 0x2E, mGreen = 0x8B, mBlue = 0x57;
                break;
            case SCARLET:
                mRed = 0xFF, mGreen = 0x24, mBlue = 0x00;
                break;
            case PUMPKIN:
                mRed = 0xFF, mGreen = 0x75, mBlue = 0x18;
                break;
            case JASMINE:
                mRed = 0xF8, mGreen = 0xDE, mBlue = 0x7E;
                break;
            case IVORY:
                mRed = 0xFF, mGreen = 0xFF, mBlue = 0xF0;
                break;
            case GAINSBORO:
                mRed = 0xDC, mGreen = 0xDC, mBlue = 0xDC;
                break;
            default:
                mRed = 0xFF, mGreen = 0xFF, mBlue = 0xFF;
                break;
        }
    }

    explicit operator float() const
    {
        PointT point(mRed, mGreen, mBlue);
        return point.rgb;
    }

    explicit operator std_msgs::ColorRGBA() const
    {
        std_msgs::ColorRGBA color;
        color.r = mRed / 255.0f;
        color.g = mGreen / 255.0f;
        color.b = mBlue / 255.0f;
        color.a = 1.0f;
        return color;
    }
};

}   // namespace mdr_cloud_object_detection

#endif  // MDR_CLOUD_OBJECT_DETECTION_COLOR_H
