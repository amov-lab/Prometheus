#ifndef MESSAGE_CONVERT_HPP
#define MESSAGE_CONVERT_HPP

/*
    用于mavlink消息流 跟 mavros消息转换
即： mavlink_message_t 跟 mavros_msgs/Mavlink 相互转换
*/
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>

namespace message_convert
{
    /**
     * @brief Convert mavros_msgs/Mavlink message to mavlink_message_t
     *
     * @note signature vector should be empty for unsigned OR
     *       MAVLINK_SIGNATURE_BLOCK size for signed messages
     *
     * @param[in]  rmsg	mavros_msgs/Mavlink message
     * @param[out] mmsg	mavlink_message_t struct
     * @return true if success
     */
    inline bool convert(const mavros_msgs::Mavlink &rmsg, mavlink_message_t &mmsg)
    {
        if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0]))
        {
            return false;
        }

        if (!rmsg.signature.empty() && rmsg.signature.size() != sizeof(mmsg.signature))
        {
            return false;
        }

        // [[[cog:
        // for f in FIELD_NAMES:
        //     cog.outl("mmsg.%s = rmsg.%s;" % (f, f))
        // ]]]
        mmsg.magic = rmsg.magic;
        mmsg.len = rmsg.len;
        mmsg.incompat_flags = rmsg.incompat_flags;
        mmsg.compat_flags = rmsg.compat_flags;
        mmsg.seq = rmsg.seq;
        mmsg.sysid = rmsg.sysid;
        mmsg.compid = rmsg.compid;
        mmsg.msgid = rmsg.msgid;
        mmsg.checksum = rmsg.checksum;
        // [[[end]]] (checksum: 2ef42a7798f261bfd367bf4157b11ec0)
        std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);
        std::copy(rmsg.signature.begin(), rmsg.signature.end(), mmsg.signature);

        return true;
    }

    /**
     * @brief Convert mavlink_message_t to mavros/Mavlink
     *
     * @param[in]  mmsg	mavlink_message_t struct
     * @param[out] rmsg	mavros_msgs/Mavlink message
     * @param[in]  framing_status  framing parse result (OK, BAD_CRC or BAD_SIGNATURE)
     * @return true, this convertion can't fail
     */
    inline bool convert(const mavlink_message_t &mmsg, mavros_msgs::Mavlink &rmsg, uint8_t framing_status = mavros_msgs::Mavlink::FRAMING_OK)
    {
        const size_t payload64_len = (mmsg.len + 7) / 8;

        rmsg.framing_status = framing_status;

        // [[[cog:
        // for f in FIELD_NAMES:
        //     cog.outl("rmsg.%s = mmsg.%s;" % (f, f))
        // ]]]
        rmsg.magic = mmsg.magic;
        rmsg.len = mmsg.len;
        rmsg.incompat_flags = mmsg.incompat_flags;
        rmsg.compat_flags = mmsg.compat_flags;
        rmsg.seq = mmsg.seq;
        rmsg.sysid = mmsg.sysid;
        rmsg.compid = mmsg.compid;
        rmsg.msgid = mmsg.msgid;
        rmsg.checksum = mmsg.checksum;
        // [[[end]]] (checksum: 4f0a50d2fcd7eb8823aea3e0806cd698)
        rmsg.payload64 = mavros_msgs::Mavlink::_payload64_type(mmsg.payload64, mmsg.payload64 + payload64_len);

        // copy signature block only if message is signed
        if (mmsg.incompat_flags & MAVLINK_IFLAG_SIGNED)
            rmsg.signature = mavros_msgs::Mavlink::_signature_type(mmsg.signature, mmsg.signature + sizeof(mmsg.signature));
        else
            rmsg.signature.clear();

        return true;
    }
}

#endif