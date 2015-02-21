/*
 * main.cpp
 *
 *  Created on: 21.02.2015
 *      Author: andi
 */
#include "util/datatypes.hpp"

#include <string.h>

extern "C" {
#include "hal_can.h"
#include "hal_eth_std_mac.h"
#include "common_func_netx500.h"
}


namespace ecpp
{
  class Can_Driver
  {
  public:
    struct Can_Frame
    {
      uint32_t id;
      uint8_t  dlc;
      uint8_t  data[8];
    };
  };

  class NetX_Can_Driver : public Can_Driver
  {
  private:
    const uint8_t _channel;
    CAN_FRAME_T   _recvbuf;

  public:
    NetX_Can_Driver(uint8_t channel);

    void Open(unsigned long BaudRate, bool listen);
    void Close();

    bool Receive(Can_Driver::Can_Frame & buf);

    void EnableStdId(uint16_t id);
    void DisableStdId(uint16_t id);

    void GetCounters(CAN_COUNTERS & counters);
    void Poll();

  };

  NetX_Can_Driver::NetX_Can_Driver(uint8_t channel) :
      _channel(channel)
  {

  }

  void NetX_Can_Driver::Open(unsigned long BaudRate, bool listen)
  {
    Can_Init(_channel, BaudRate, listen, 0, 0);
    Can_Start(_channel, 0);
  }

  void NetX_Can_Driver::Close()
  {
    Can_Deinit(_channel, 0);
  }

  void NetX_Can_Driver::EnableStdId(uint16_t id)
  {
    Can_EnDisRxStdId(_channel, 1, id);
  }

  void NetX_Can_Driver::DisableStdId(uint16_t id)
  {
    Can_EnDisRxStdId(_channel, 0, id);
  }

  bool NetX_Can_Driver::Receive(Can_Driver::Can_Frame & buf)
  {
    CAN_RESULT result;


    /* check for high prio can frame */
    result = Can_ReceiveFrame(_channel, 1, 0, &_recvbuf);

    /* check for low prio can frame */
    if (CAN_OKAY != result)
      result = Can_ReceiveFrame(_channel, 0, 0, &_recvbuf);

    if (CAN_OKAY == result)
    {
      buf.dlc  = _recvbuf.ulFrameInfo & 0xF;
      buf.id   = _recvbuf.ulIdentifier;
      memcpy(buf.data,_recvbuf.abData, sizeof(buf.data));
    }
    else
    {
      buf.dlc=0;
    }

    return (CAN_OKAY == result);
  }

  void NetX_Can_Driver::GetCounters(CAN_COUNTERS & counters)
  {
    Can_GetCounters(_channel, &counters);
  }

  void NetX_Can_Driver::Poll()
  {
    unsigned long irq;

    /* if we somehow get bus off, we must confirm irq to reenable bus */
    Can_BusOffAcknowledge(_channel);
  }

  class NetX_Eth_StdMac_Driver
  {
  public:
    class NetX_Eth_StdMac_FrameBuffer
    {
    public:
      union {
        ETHERNET_FRAME_T *ptFrame;
        uint8_t          *pbFrame;
      };
      void*             hFrame;
      uint16_t          length;
    };
  private:
    const uint8_t _channel;
    const uint8_t _macaddr[6];

  public:
    NetX_Eth_StdMac_Driver(uint8_t channel, const uint8_t (&macaddr)[6]);

    void Open();
    void Close();

    bool GetBuffer(NetX_Eth_StdMac_FrameBuffer &buf);

    void SendBuffer(NetX_Eth_StdMac_FrameBuffer &buf);

    void Poll();
  };

  NetX_Eth_StdMac_Driver::NetX_Eth_StdMac_Driver(uint8_t channel, const uint8_t (&macaddr)[6]) :
      _channel(channel), _macaddr{macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]}
  {

  }

  void NetX_Eth_StdMac_Driver::Open()
  {

    NX500_PHY_CONTROL_T phyctrl;
    phyctrl.val = 0;

    if(_channel == 0)
    {
      phyctrl.bf.phy0_automdix = 1;
      phyctrl.bf.phy0_enable   = 1;
      phyctrl.bf.phy0_mode     = PHYCTRL_ALL_CAPABLE_AUTONEG_AUTOMDIXEN;
    }
    else if (_channel == 1)
    {
      phyctrl.bf.phy1_automdix = 1;
      phyctrl.bf.phy1_enable   = 1;
      phyctrl.bf.phy1_mode     = PHYCTRL_ALL_CAPABLE_AUTONEG_AUTOMDIXEN;
    }

    NX500_INTPHY_Init( &phyctrl);

    EthStdMac_Initialize(_channel, ETH_PHY_LED_BLINK, 5000000, NULL);
    EthStdMac_Start(_channel, NULL);
  }

  bool NetX_Eth_StdMac_Driver::GetBuffer(NetX_Eth_StdMac_FrameBuffer &buf)
  {
    ETHERNET_RESULT result;

    result = EthStdMac_GetFrame(_channel, &(buf.ptFrame), &(buf.hFrame));

    if (ETH_OKAY == result)
    {
      memcpy(&(buf.ptFrame->tSrcAddr), _macaddr, sizeof(buf.ptFrame->tSrcAddr));
    }

    return (ETH_OKAY == result);
  }

  void NetX_Eth_StdMac_Driver::SendBuffer(NetX_Eth_StdMac_FrameBuffer &buf)
  {
    ETHERNET_RESULT result;
    int length;

    length = buf.length;

    if(length < 60)
    {
      memset(buf.pbFrame + length, 0x00, 60 -length);
      length = 60;
    }

    result = EthStdMac_Send(_channel, buf.hFrame, length, 0);

    if (ETH_OKAY != result)
      EthStdMac_ReleaseFrame(_channel, buf.hFrame);
  }

  void NetX_Eth_StdMac_Driver::Poll()
  {
    ETHERNET_RESULT result, result2;
    NetX_Eth_StdMac_FrameBuffer buf;
    unsigned long length, irq;

    EthStdMac_GetIrq(_channel, &irq);
    EthStdMac_ConfirmIrq(_channel, irq);

    result = EthStdMac_Recv(_channel, &(buf.ptFrame), &(buf.hFrame), &length, 1);

    if(ETH_OKAY == result)
      EthStdMac_ReleaseFrame(_channel, buf.hFrame);

    result = EthStdMac_Recv(_channel, &(buf.ptFrame), &(buf.hFrame), &length, 0);

    if(ETH_OKAY == result)
      EthStdMac_ReleaseFrame(_channel, buf.hFrame);

    result = EthStdMac_GetSendCnf(_channel, &(buf.ptFrame), &(buf.hFrame), &length, 0, &result2);

    if(ETH_OKAY == result)
      EthStdMac_ReleaseFrame(_channel, buf.hFrame);

    result = EthStdMac_GetSendCnf(_channel, &(buf.ptFrame), &(buf.hFrame), &length, 1, &result2);

    if(ETH_OKAY == result)
      EthStdMac_ReleaseFrame(_channel, buf.hFrame);

  }
}

uint8_t hton8(uint8_t value)
{
  return value;
}

uint16_t hton16(uint16_t value)
{
  return (value >> 8) | (value << 8);
}

uint32_t hton32(uint32_t value)
{
  return (value >> 24) | ((value >> 8) & 0x0000FF00) | ((value << 8) & 0x00FF0000)| (value << 24);
}


struct __attribute__((packed,aligned(4))) can_pdu
{
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
};

using namespace ecpp;

int main()
{
  const uint8_t macaddr[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

  NetX_Can_Driver can(3);
  NetX_Eth_StdMac_Driver eth(0, macaddr);

  NX500_IRQFIQ_LOCK;

  NX500_XC_Reset(0,0);
  NX500_XC_Reset(1,0);
  NX500_XC_Reset(2,0);
  NX500_XC_Reset(3,0);

  NX500_PFIFO_Reset();

  can.Open(125000, false);
  eth.Open();

  /* enable can reception of all ids */
  for(int i = 0; i <= CAN_MAX_STD_IDENTIFIER; ++i)
    can.EnableStdId(i);

  unsigned long seconds_last, nanoseconds_last;

  NX500_SYSTIME_SetBorder(DFLT_VAL_NX500_systime_border_systime_border, NULL);
  NX500_SYSTIME_SetSpeed(DFLT_VAL_NX500_systime_count_value, NULL);

  NX500_SYSTIME_SetSystime(0,0,0);
  NX500_SYSTIME_GetSystime(&seconds_last, &nanoseconds_last, NULL);
  seconds_last -= 1;

  while(true)
  {
    NetX_Can_Driver::Can_Frame                          canfrm;

    if (can.Receive(canfrm))
    {
      NetX_Eth_StdMac_Driver::NetX_Eth_StdMac_FrameBuffer ethbuf;

      if (eth.GetBuffer(ethbuf))
      {
        memset(&(ethbuf.ptFrame->tDstAddr), 0xFF, sizeof((ethbuf.ptFrame->tDstAddr)));
        ethbuf.ptFrame->usType = hton16((uint16_t)(0x88b5));

        struct can_pdu *pdu = reinterpret_cast<struct can_pdu*>(&(ethbuf.ptFrame->abData[2]));
        pdu->id = hton32(canfrm.id);
        pdu->dlc = hton8(canfrm.dlc);
        memcpy(pdu->data, canfrm.data, canfrm.dlc);
        memset(pdu->data + canfrm.dlc, 0x00, sizeof(pdu->data) - canfrm.dlc);

        ethbuf.length = reinterpret_cast<uint8_t*>(pdu + 1) - ethbuf.pbFrame;

        eth.SendBuffer(ethbuf);
      }
    }

    unsigned long seconds, nanoseconds;
    NX500_SYSTIME_GetSystime(&seconds, &nanoseconds, NULL);

    if(seconds != seconds_last)
    {
      seconds_last     = seconds;
      nanoseconds_last = nanoseconds;

      NetX_Eth_StdMac_Driver::NetX_Eth_StdMac_FrameBuffer ethbuf;

      if (eth.GetBuffer(ethbuf))
      {
        memset(&(ethbuf.ptFrame->tDstAddr), 0xFF, sizeof((ethbuf.ptFrame->tDstAddr)));
        ethbuf.ptFrame->usType = hton16((uint16_t)(0x88b6));

        CAN_COUNTERS *counters = reinterpret_cast<CAN_COUNTERS*>(&(ethbuf.ptFrame->abData[2]));

        can.GetCounters(*counters);

        ethbuf.length = reinterpret_cast<uint8_t*>(counters + 1) - ethbuf.pbFrame;

        eth.SendBuffer(ethbuf);
      }

    }

    eth.Poll();
    can.Poll();
  }
  return 0;
}

