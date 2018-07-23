/*********************************************************************************
File name:	  CLidarPacket.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-04
Description:  lidar packet
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_CLIDARPACKET_H
#define EVEREST_LIDAR_CLIDARPACKET_H

/******************************* Current libs includes ****************************/
#include "typedef.h"

/******************************* System libs includes *****************************/
#include <vector>
#include <stddef.h>


namespace everest
{
	namespace hwdrivers
	{
	    enum TLidarDeviceType
	    {
	        LDSUNKNOW = -1,
            LDS2K = 0,
            LDSBS = 1,
	    };

	    class CLidarPacket
	    {
	        public:
	            enum TCheckNumberType
	            {
                    CHECK_NUMBER_CRC = 0,
                    CHECK_NUMBER_SUM
	            };

                /* Constructor */
                CLidarPacket();

                /* Destructor */
                ~CLidarPacket() { }

                /* Return true if buffer is empty */
                bool isEmpty() { return m_length == 0? true: false; }

                /* Return buffer size */
                u16 getSize() const { return m_length; }

                /* Return true when crc verify good */
                bool isValid() const { return m_valid; }

                /* Push ch in buffer end */
                void pushBack(u8 ch);

                /* Reset packet */
                void reset();

                /* return true if has write capacity */
				bool hasWriteCapacity(int bytes);

				/* CRC sum calculate */
				u16 calcCheckSumCRC(u8 *start_bytes, u16 num_bytes);
				u16 calcCheckSumSum(u8 *start_bytes, u16 num_bytes);
				bool verifyCheckSum(TCheckNumberType type);

				/* Print hex */
                void printHex();

                /* Get address code */
                u8 getAddressCode() const { return m_buf[3]; }

                /* Get frame type */
                u8 getFrameType() const { return isValid()? m_buf[4]: 0xFF; }

                /* Get command ID */
                u8 getCommandID() const { return isValid()? m_buf[5]: 0xFF; }

                /* Get params data ptr */
                u8* getParamPtr() {return isValid()? (&m_buf[8]): NULL; }

                /* Get params data length */
                u16 getParamLength();

                /* Get lidar device type */
                TLidarDeviceType getLidarDeviceType() {return m_lidar_type;}

                /* buf to u16 */
                static u16 bufToUByte2(u8 *src_ptr);

                /* buf to s16 */
                static s16 bufToByte2(u8 *src_ptr);

                /* buf to u8 */
                static u8 bufToUByte(u8 *src_ptr);

                /* Buffer to data */
                static void bufferToData(void *dest_ptr, void *src_ptr, size_t length);

                struct TParams
                {
                    TParams()
                    {
                        buf_size = 512;
                        // 帧头 帧长度 地址码 帧类型 命令字 参数长度 参数 CRC16
                        //  1    2     1     1     1     2           2
                        //  当帧类型为命令帧时，参数长度以及参数可为0，因此最小帧长度为
                        //  1 +  2  +  1  +  1  +  1  +  0  +  0  +  2 = 8
                        least_packet_len = 8;
                    }

                    u16 buf_size;
                    u16 least_packet_len;
                };
            private:
                u8* getBufPtr() { return &m_buf[0]; }

            public:
                std::vector<u8>    m_buf;
                u16                m_read_length;
                u16                m_length;
                TParams            m_params;
                bool               m_valid;
                TLidarDeviceType   m_lidar_type;
	    };
	}
}

#endif


