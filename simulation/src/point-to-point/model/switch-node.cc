#include "ns3/ipv4.h"
#include "ns3/packet.h"
#include "ns3/ipv4-header.h"
#include "ns3/pause-header.h"
#include "ns3/flow-id-tag.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "switch-node.h"
#include "qbb-net-device.h"
#include "ppp-header.h"
#include "ns3/int-header.h"
#include <ns3/seq-ts-header.h>
#include <cmath>
#include <string>

namespace ns3 {

TypeId SwitchNode::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SwitchNode")
    .SetParent<Node> ()
    .AddConstructor<SwitchNode> ()
	.AddAttribute("EcnEnabled",
			"Enable ECN marking.",
			BooleanValue(false),
			MakeBooleanAccessor(&SwitchNode::m_ecnEnabled),
			MakeBooleanChecker())
	.AddAttribute("CcMode",
			"CC mode.",
			UintegerValue(0),
			MakeUintegerAccessor(&SwitchNode::m_ccMode),
			MakeUintegerChecker<uint32_t>())
	.AddAttribute("AckHighPrio",
			"Set high priority for ACK/NACK or not",
			UintegerValue(0),
			MakeUintegerAccessor(&SwitchNode::m_ackHighPrio),
			MakeUintegerChecker<uint32_t>())
	.AddAttribute("MaxRtt",
			"Max Rtt of the network",
			UintegerValue(9000),
			MakeUintegerAccessor(&SwitchNode::m_maxRtt),
			MakeUintegerChecker<uint32_t>())
  ;
  return tid;
}

SwitchNode::SwitchNode(){
	m_ecmpSeed = m_id;
	m_node_type = 1;
	m_mmu = CreateObject<SwitchMmu>();
	for (uint32_t i = 0; i < pCnt; i++)
		for (uint32_t j = 0; j < pCnt; j++)
			for (uint32_t k = 0; k < qCnt; k++)
				m_bytes[i][j][k] = 0;
	for (uint32_t i = 0; i < pCnt; i++) {
		m_txBytes[i] = 0;
		m_lastPktSize[i] = m_lastPktTs[i] = 0;
		m_u[i] = 0;

		countStart[i] = true;

		low_rate[i] = 10;
		high_rate[i] = 0;
		mid_rate[i] = 0;
		m_fair[i] = 0;
		
		for (uint32_t j = 0; j < 8; j++){
			rateCnt[i][j] = 0;
			rateCntRes[i][j] = 0;
		}

	}
	count_T = 8192;
	
}

int SwitchNode::GetOutDev(Ptr<const Packet> p, CustomHeader &ch){
	// look up entries
	auto entry = m_rtTable.find(ch.dip);

	// no matching entry
	if (entry == m_rtTable.end())
		return -1;

	// entry found
	auto &nexthops = entry->second;

	// pick one next hop based on hash
	union {
		uint8_t u8[4+4+2+2];
		uint32_t u32[3];
	} buf;
	buf.u32[0] = ch.sip;
	buf.u32[1] = ch.dip;
	if (ch.l3Prot == 0x6)
		buf.u32[2] = ch.tcp.sport | ((uint32_t)ch.tcp.dport << 16);
	else if (ch.l3Prot == 0x11)
		buf.u32[2] = ch.udp.sport | ((uint32_t)ch.udp.dport << 16);
	else if (ch.l3Prot == 0xFC || ch.l3Prot == 0xFD)
		buf.u32[2] = ch.ack.sport | ((uint32_t)ch.ack.dport << 16);
	uint32_t idx = EcmpHash(buf.u8, 12, m_ecmpSeed) % nexthops.size();

	return nexthops[idx];
}

void SwitchNode::CheckAndSendPfc(uint32_t inDev, uint32_t qIndex){
	//std::cout<<qIndex<<'\n';
	Ptr<QbbNetDevice> device = DynamicCast<QbbNetDevice>(m_devices[inDev]);
	if (m_mmu->CheckShouldPause(inDev, qIndex)){
		device->SendPfc(qIndex, 0);
		m_mmu->SetPause(inDev, qIndex);
	}
}
void SwitchNode::CheckAndSendResume(uint32_t inDev, uint32_t qIndex){
	Ptr<QbbNetDevice> device = DynamicCast<QbbNetDevice>(m_devices[inDev]);
	if (m_mmu->CheckShouldResume(inDev, qIndex)){
		device->SendPfc(qIndex, 1);
		m_mmu->SetResume(inDev, qIndex);
	}
}

void SwitchNode::SendToDev(Ptr<Packet>p, CustomHeader &ch) {
	int idx = GetOutDev(p, ch);
	// if(ch.sip == 0x0b000801 && ch.udp.seq <= 10000){
	// 	printf("%u ,", this->GetId());
	// }
	// if(this->GetId() == 1025){
	// 	if(ch.sip == 0x0b000801 || ch.sip == 0x0b000901){
	// 		idx = 9;
	// 	}
	// 	if(ch.sip >= 0x0b000a01 && ch.sip <= 0x0b000f01){
	// 		idx != 9;
	// 	}
	// }
	if (idx >= 0){
		NS_ASSERT_MSG(m_devices[idx]->IsLinkUp(), "The routing table look up should return link that is up");

		// determine the qIndex
		uint32_t qIndex;
		if (ch.l3Prot == 0xFF || ch.l3Prot == 0xFE || (m_ackHighPrio && (ch.l3Prot == 0xFD || ch.l3Prot == 0xFC))){  //QCN or PFC or NACK, go highest priority
			qIndex = 0;
		}else{
			qIndex = (ch.l3Prot == 0x06 ? 1 : ch.udp.pg); // if TCP, put to queue 1
		}
		// admission control
		FlowIdTag t;
		p->PeekPacketTag(t);
		uint32_t inDev = t.GetFlowId();		//需要进入哪个端口
		if (qIndex != 0){ //not highest priority
			if (m_mmu->CheckIngressAdmission(inDev, qIndex, p->GetSize()) && m_mmu->CheckEgressAdmission(idx, qIndex, p->GetSize())){			// Admission control
				m_mmu->UpdateIngressAdmission(inDev, qIndex, p->GetSize());
				m_mmu->UpdateEgressAdmission(idx, qIndex, p->GetSize());
			}else{
				return; // Drop
			}
			CheckAndSendPfc(inDev, qIndex);
		}
		m_bytes[inDev][idx][qIndex] += p->GetSize();
		m_devices[idx]->SwitchSend(qIndex, p, ch);

			//从统计区间第一个包开始，只统计count_T时间内的包，每过count_T的时间重置rateCnt中的包个数统计
			if(countStart[idx]){
				Simulator::Schedule(NanoSeconds(count_T), &SwitchNode::countInterval, this, idx);
				countStart[idx] = false;
			}
			//提取当前包携带的速率
			//取消申请机制，让交换机只负责记录histogram
			uint8_t pos = ch.udp.cur_rate;
			rateCnt[idx][pos]++;

	}else{
		return; // Drop
	}
		
}

uint32_t SwitchNode::EcmpHash(const uint8_t* key, size_t len, uint32_t seed) {
  uint32_t h = seed;
  if (len > 3) {
    const uint32_t* key_x4 = (const uint32_t*) key;
    size_t i = len >> 2;
    do {
      uint32_t k = *key_x4++;
      k *= 0xcc9e2d51;
      k = (k << 15) | (k >> 17);
      k *= 0x1b873593;
      h ^= k;
      h = (h << 13) | (h >> 19);
      h += (h << 2) + 0xe6546b64;
    } while (--i);
    key = (const uint8_t*) key_x4;
  }
  if (len & 3) {
    size_t i = len & 3;
    uint32_t k = 0;
    key = &key[i - 1];
    do {
      k <<= 8;
      k |= *key--;
    } while (--i);
    k *= 0xcc9e2d51;
    k = (k << 15) | (k >> 17);
    k *= 0x1b873593;
    h ^= k;
  }
  h ^= len;
  h ^= h >> 16;
  h *= 0x85ebca6b;
  h ^= h >> 13;
  h *= 0xc2b2ae35;
  h ^= h >> 16;
  return h;
}

void SwitchNode::SetEcmpSeed(uint32_t seed){
	//printf("%ld\n",seed);
	m_ecmpSeed = seed;
}

void SwitchNode::AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx){
	uint32_t dip = dstAddr.Get();
	m_rtTable[dip].push_back(intf_idx);
}

void SwitchNode::ClearTable(){
	m_rtTable.clear();
}

// This function can only be called in switch mode(node?)
bool SwitchNode::SwitchReceiveFromDevice(Ptr<NetDevice> device, Ptr<Packet> packet, CustomHeader &ch){
	SendToDev(packet, ch);
	return true;
}

void SwitchNode::SwitchNotifyDequeue(uint32_t ifIndex, uint32_t qIndex, Ptr<Packet> p){ //egress逻辑
	FlowIdTag t;
	p->PeekPacketTag(t);
	//mmu
	if (qIndex != 0){
		uint32_t inDev = t.GetFlowId();
		m_mmu->RemoveFromIngressAdmission(inDev, qIndex, p->GetSize());
		m_mmu->RemoveFromEgressAdmission(ifIndex, qIndex, p->GetSize());
		m_bytes[inDev][ifIndex][qIndex] -= p->GetSize();
		if (m_ecnEnabled){
			bool egressCongested = m_mmu->ShouldSendCN(ifIndex, qIndex);
			if (egressCongested){
				PppHeader ppp;
				Ipv4Header h;
				p->RemoveHeader(ppp);
				p->RemoveHeader(h);
				h.SetEcn((Ipv4Header::EcnType)0x03);
				p->AddHeader(h);
				p->AddHeader(ppp);
			}
		}
		
		//CheckAndSendPfc(inDev, qIndex);
		CheckAndSendResume(inDev, qIndex);
	}
	//INT
	if (1){

		uint8_t* buf = p->GetBuffer();
		if (buf[PppHeader::GetStaticSize() + 9] == 0x11){ // udp packet
			
			//现在是利用的ch读取头部，是否可以跟INT一样，读取到特定位置呢
			CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
			ch.getInt = 1; // parse INT header

			IntHeader *ih = (IntHeader*)&buf[PppHeader::GetStaticSize() + 20 + 8 + 7]; // ppp, ip, udp, SeqTs, INT
			Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(m_devices[ifIndex]);
			if (m_ccMode == 3){ // HPCC
				//接下来想在这个INT部分，针对不同的速率做出一定的调整
				uint32_t qlen = dev->GetQueue()->GetNBytesTotal();
				//这里不采用成倍增加，因为在qlen = 0的情况下成倍还是0无法产生影响
				//if(pos >= 7) qlen += 20 * 1090;
				//if(pos > low_rate[ifIndex]) qlen = qlen + 5 * 1092;
				ih->PushHop(Simulator::Now().GetTimeStep(), m_txBytes[ifIndex], qlen, dev->GetDataRate().GetBitRate(), rateCntRes[ifIndex]);
			}else if (m_ccMode == 10){ // HPCC-PINT
				uint64_t t = Simulator::Now().GetTimeStep();
				uint64_t dt = t - m_lastPktTs[ifIndex];
				if (dt > m_maxRtt)
					dt = m_maxRtt;
				uint64_t B = dev->GetDataRate().GetBitRate() / 8; //Bps
				uint64_t qlen = dev->GetQueue()->GetNBytesTotal();
				double newU;

				/**************************
				 * approximate calc
				 *************************/
				int b = 20, m = 16, l = 20; // see log2apprx's paremeters
				int sft = logres_shift(b,l);
				double fct = 1<<sft; // (multiplication factor corresponding to sft)
				double log_T = log2(m_maxRtt)*fct; // log2(T)*fct
				double log_B = log2(B)*fct; // log2(B)*fct
				double log_1e9 = log2(1e9)*fct; // log2(1e9)*fct
				double qterm = 0;
				double byteTerm = 0;
				double uTerm = 0;
				if ((qlen >> 8) > 0){
					int log_dt = log2apprx(dt, b, m, l); // ~log2(dt)*fct
					int log_qlen = log2apprx(qlen >> 8, b, m, l); // ~log2(qlen / 256)*fct
					qterm = pow(2, (
								log_dt + log_qlen + log_1e9 - log_B - 2*log_T
								)/fct
							) * 256;
					// 2^((log2(dt)*fct+log2(qlen/256)*fct+log2(1e9)*fct-log2(B)*fct-2*log2(T)*fct)/fct)*256 ~= dt*qlen*1e9/(B*T^2)
				}
				if (m_lastPktSize[ifIndex] > 0){
					int byte = m_lastPktSize[ifIndex];
					int log_byte = log2apprx(byte, b, m, l);
					byteTerm = pow(2, (
								log_byte + log_1e9 - log_B - log_T
								)/fct
							);
					// 2^((log2(byte)*fct+log2(1e9)*fct-log2(B)*fct-log2(T)*fct)/fct) ~= byte*1e9 / (B*T)
				}
				if (m_maxRtt > dt && m_u[ifIndex] > 0){
					int log_T_dt = log2apprx(m_maxRtt - dt, b, m, l); // ~log2(T-dt)*fct
					int log_u = log2apprx(int(round(m_u[ifIndex] * 8192)), b, m, l); // ~log2(u*512)*fct
					uTerm = pow(2, (
								log_T_dt + log_u - log_T
								)/fct
							) / 8192;
					// 2^((log2(T-dt)*fct+log2(u*512)*fct-log2(T)*fct)/fct)/512 = (T-dt)*u/T
				}
				newU = qterm+byteTerm+uTerm;

				#if 0
				/**************************
				 * accurate calc
				 *************************/
				double weight_ewma = double(dt) / m_maxRtt;
				double u;
				if (m_lastPktSize[ifIndex] == 0)
					u = 0;
				else{
					double txRate = m_lastPktSize[ifIndex] / double(dt); // B/ns
					u = (qlen / m_maxRtt + txRate) * 1e9 / B;
				}
				newU = m_u[ifIndex] * (1 - weight_ewma) + u * weight_ewma;
				printf(" %lf\n", newU);
				#endif

				/************************
				 * update PINT header
				 ***********************/
				uint16_t power = Pint::encode_u(newU);
				if (power > ih->GetPower())
					ih->SetPower(power);

				m_u[ifIndex] = newU;
			}
		}
	}

	//每转发一个包会把这个packet size统计到txBytes中去
	m_txBytes[ifIndex] += p->GetSize();
	//for pint
	m_lastPktSize[ifIndex] = p->GetSize();
	m_lastPktTs[ifIndex] = Simulator::Now().GetTimeStep();

}

void SwitchNode::countInterval(uint32_t p){
	//先打印出来观察数据情况
	
	//if(this->GetId() == 6 && p == 3 && Simulator::Now().GetTimeStep() - 2000000000 < 5000000) printf("%lu, ", Simulator::Now().GetTimeStep() - 2000000000);

	for(uint8_t i = 0; i < 8; i++){
		//目前采取的操作是将结果复制到另一个数组中，原数组继续
		rateCntRes[p][i] = rateCnt[p][i];
		rateCnt[p][i] = 0;

		//if(this->GetId() == 6 && p == 3 && Simulator::Now().GetTimeStep() - 2000000000 < 5000000) printf("%u, ", rateCntRes[p][i]);
	}

	//能够对这个数组做什么样的分析呢？
	//目前包括公平性、最低速率、最高速率
	// uint8_t dis = 0;	//分布，有几根柱子
	// uint8_t low = 9;	//最低速率
	// uint8_t high = 0;	//最高速率
	// for(uint8_t i = 0; i < 8; i++){
	// 	if(rateCntRes[p][i] >= 10){
	// 		dis++;
	// 		if(i < low) low = i;
	// 		if(i > high) high = i;
	// 	}
	// }
	// if(dis <= 1) m_fair[p] = 1;
	// low_rate[p] = low;
	// high_rate[p] = high;

	//if(this->GetId() == 6 && p == 3 && Simulator::Now().GetTimeStep() - 2000000000 < 5000000) printf("\n");

	Simulator::Schedule(NanoSeconds(count_T), &SwitchNode::countInterval, this, p);
}


int SwitchNode::logres_shift(int b, int l){
	static int data[] = {0,0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
	return l - data[b];
}

int SwitchNode::log2apprx(int x, int b, int m, int l){
	int x0 = x;
	int msb = int(log2(x)) + 1;
	if (msb > m){
		x = (x >> (msb - m) << (msb - m));
		#if 0
		x += + (1 << (msb - m - 1));
		#else
		int mask = (1 << (msb-m)) - 1;
		if ((x0 & mask) > (rand() & mask))
			x += 1<<(msb-m);
		#endif
	}
	return int(log2(x) * (1<<logres_shift(b, l)));
}

/*void SwitchNode::updateRtCnt(bool first, int p){

	//打印这一阶段的数据信息
	//第一波，只计数，不复制
	//后续，清零，数据复制到旧数组，这里可以循环利用数组
	//在4个表之间循环更新
	if(!first){
		rateIdx[p] = (rateIdx[p] + 1) % 4;
		if(this->GetId() == 5 && p == 3) printf("%lu %u ", Simulator::Now().GetTimeStep() - 2000000000, rateIdx[p]);
		for (uint32_t i = 0; i < 10; i++){
			if(rateIdx[p] == 1){
				rateCntOld1[p][i] = rateCnt[p][i];
				if(this->GetId() == 5 && p == 3) printf("%lu, ", rateCntOld1[p][i]);
			}else if(rateIdx[p] == 2){
				rateCntOld2[p][i] = rateCnt[p][i];
				if(this->GetId() == 5 && p == 3) printf("%lu, ", rateCntOld2[p][i]);
			}else if(rateIdx[p] == 3){
				rateCntOld3[p][i] = rateCnt[p][i];
				if(this->GetId() == 5 && p == 3) printf("%lu, ", rateCntOld3[p][i]);
			}else if(rateIdx[p] == 0){
				rateCntOld4[p][i] = rateCnt[p][i];
				if(this->GetId() == 5 && p == 3) printf("%lu, ", rateCntOld4[p][i]);
			}
			rateCnt[p][i] = 0;
		}
		if(isSteady(p)) {
			if(this->GetId() == 5 && p == 3) printf("yes");
			m_fair[p] = true;
		}else{
			if(this->GetId() == 5 && p == 3) printf("no");
			m_fair[p] = false;
		}
		if(this->GetId() == 5 && p == 3) printf("\n");
		
	}

	//下一次调用
	//if(Simulator::Now().GetTimeStep() - 2000000000 <= 100000000)
		Simulator::Schedule(NanoSeconds(count_T), &SwitchNode::updateRtCnt, this, false, p);
}*/

//steady也需要分端口考虑
/*bool SwitchNode::isSteady(uint32_t p){
	// for(uint32_t i = 0; i < 10; i++){
	// 	if(rateCntOld1[p][i] <= 2 && rateCntOld2[p][i] <= 2 && rateCntOld3[p][i] <= 2 && rateCntOld4[p][i] <= 2){
	// 		continue;
	// 	}else if(rateCntOld1[p][i] > 2 && rateCntOld2[p][i] > 2 && rateCntOld3[p][i] > 2 && rateCntOld4[p][i] > 2){
	// 		continue;
	// 	}else{
	// 		return false;
	// 	}
	// }
	//return true;
	uint16_t packetNum = packetCount(p);
	Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(m_devices[p]);
	if(packetNum < 170) return false;
	if(packetNum >= 170 && dev->GetQueue()->GetNBytesTotal() <= 5 * 1106)
		return true;
	return false;
}*/

// int SwitchNode::slowestPos(uint64_t rateCnt[]){
// 	for(int i = 0; i < 10; ++i){
// 		if(rateCnt[i] > 2 ){
// 			return i;
// 		}
// 	}
// }

// int SwitchNode::highestPos(uint32_t p){
// 	int high = 0;
// 	if(rateIdx[p] == 1) high = highest(rateCntOld1[p]);
// 	if(rateIdx[p] == 2) high = highest(rateCntOld2[p]);
// 	if(rateIdx[p] == 3) high = highest(rateCntOld3[p]);
// 	if(rateIdx[p] == 0) high = highest(rateCntOld4[p]);
// 	return high;
// }

// int SwitchNode::highest(uint64_t rateCnt[]){
// 	int pos = 0;
// 	for(int i = 0; i < 10; ++i){
// 		if(rateCnt[i] > 2 ){
// 			pos = i;
// 		}
// 	}
// 	return pos;
// }

// int SwitchNode::existPos(uint64_t rateCnt[]){
// 	int exist = 0;
// 	for(int i = 0; i < 10; ++i){
// 		if(rateCnt[i] > 2 ){
// 			exist++;
// 		}
// 	}
// 	return exist;
// }

/*void SwitchNode::checkFair(uint32_t p){
	//if(this->GetId() == 5 && p == 3) printf("%u checking\n", stage[p]);
	if(m_fair[p] == true){
		int slowest = 0;
			if(rateIdx[p] == 1) slowest = slowestPos(rateCntOld1[p]);
			if(rateIdx[p] == 2) slowest = slowestPos(rateCntOld2[p]);
			if(rateIdx[p] == 3) slowest = slowestPos(rateCntOld3[p]);
			if(rateIdx[p] == 0) slowest = slowestPos(rateCntOld4[p]);
		int exist = 0;
			if(rateIdx[p] == 1) exist = existPos(rateCntOld1[p]);
			if(rateIdx[p] == 2) exist = existPos(rateCntOld2[p]);
			if(rateIdx[p] == 3) exist = existPos(rateCntOld3[p]);
			if(rateIdx[p] == 0) exist = existPos(rateCntOld4[p]);
		if(exist > 1){
			target_pos[p] = slowest;
		}else{
			target_pos[p] = -1;
			stage[p] = 1;
		}
	}else{
		target_pos[p] = -1;
		stage[p] = 1;
	}
	
	//if(Simulator::Now().GetTimeStep() - 2000000000 <= 100000000){
		Simulator::Schedule(NanoSeconds(fair_T * stage[p]), &SwitchNode::checkFair, this, p);
		Simulator::Schedule(NanoSeconds(count_T), &SwitchNode::controlSpeed, this, p);
	//}

	//if(target_pos[p] != -1 && stage[p] < 16) stage[p] = 2 * stage[p];
	
}*/

/*void SwitchNode::controlSpeed(uint32_t p){
	target_pos[p] = -1;
}*/

/*uint16_t SwitchNode::packetCount(uint32_t p){
	uint16_t sum = 0;
	if(rateIdx[p] == 1){
		for(int i = 0; i < 10; i++) sum += rateCntOld1[p][i];
	}else if(rateIdx[p] == 2){
		for(int i = 0; i < 10; i++) sum += rateCntOld2[p][i];
	}else if(rateIdx[p] == 3){
		for(int i = 0; i < 10; i++) sum += rateCntOld3[p][i];
	}else if(rateIdx[p] == 0){
		for(int i = 0; i < 10; i++) sum += rateCntOld4[p][i];
	}
	return sum;
}*/

// void SwitchNode::updateFlowState(uint32_t p){
// 	newFlow[p] = false;
// }

/*void SwitchNode::updateConfig(uint32_t p){
	config[p] = !config[p];
	//std::cout<<config[p];
	if(Simulator::Now().GetTimeStep() - 2000000000 <= 15000000){
		if(config[p] == true){
			Simulator::Schedule(NanoSeconds(16384), &SwitchNode::updateConfig, this, p);
		}else{
			Simulator::Schedule(NanoSeconds(131072), &SwitchNode::updateConfig, this, p);
		}
	}
	
		
}*/

/*void SwitchNode::isFlowSteady(uint32_t idx){
	if(flows[idx] == flows_old[idx]){
		flowsteady[idx] = true;
	}else{
		flowsteady[idx] = false;
	}
	flows_old[idx] = flows[idx];
	if(Simulator::Now().GetTimeStep() - 2000000000 <= 1000000)
		Simulator::Schedule(NanoSeconds(8192), &SwitchNode::isFlowSteady, this, idx);
}*/

/*void SwitchNode::updateSketch(){

	//同时，重置Sketch中的数据
	tcm_new->init(2);
	//tcm_new->showOut();
	if(Simulator::Now().GetTimeStep() - 2000000000 <= 600000)
		Simulator::Schedule(NanoSeconds(32000), &SwitchNode::updateSketch, this);
}*/

/*void SwitchNode::updateLFT(){

	//需要在这里查看long flow是否在一段时间内没有被访问过
	//如果没有，将其从表中移除
	for (auto& flow : LFT) {
        if (flow.second == false) {
        	LFT.erase(flow.first);
        }else{
			LFT[flow.first] = false;
		}
    }

	if(Simulator::Now().GetTimeStep() - 2000000000 <= 600000)
		Simulator::Schedule(NanoSeconds(16000), &SwitchNode::updateLFT, this);
}*/

} /* namespace ns3 */
