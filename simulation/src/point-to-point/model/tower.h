#ifndef _TOWER_H
#define _TOWER_H

#include <random>
#include <cstring>
#include <algorithm>

#include "params.h"
#include "murmur3.h"

namespace ns3{
class TowerSketch
{
public:
    uint32_t w[d];
    uint32_t *A[d];
    uint32_t hashseed[d];
    int idx[d];

    TowerSketch() {}
    TowerSketch(uint32_t w_d) { init(w_d); }
    TowerSketch(TowerSketch *old){ init(old); }
    virtual ~TowerSketch() { clear(); }

    void init(uint32_t w_d)
    {
        std::random_device rd;          //随机数种子，用于生成随机数
        for (int i = 0; i < d; ++i)
        {
            w[i] = w_d << 5 - i - 1;    //w[0]=10000,w[1]=5000,w[2]=2500,w[3]=1250,w[4]=625
            A[i] = new uint32_t[w_d];   //每个A都是625个的数组？但是感觉这样并不Tower？但是最后结果是tower的，如何实现的呢
            memset(A[i], 0, w_d * sizeof(uint32_t));
            hashseed[i] = rd() % MAX_PRIME32;
        }
    }
    void init(TowerSketch *old)
    {
        for(int i = 0; i < d; ++i)
        {
            w[i] = old->w[i];
            for(int j = 0; j < w[i]; ++j)
            {
                A[i][j] = old->A[i][j];
            }
            hashseed[i] = old->hashseed[i];
        }
    }

    void showOut(){
        for (int i = 0; i < d; ++i){
            for(int j = 0; j < w[i]; j++){
                uint32_t ret = UINT32_MAX;
                uint32_t a = A[i][j >> cpw[i]];
                uint32_t shift = (j & lo[i]) << cs[i];
                uint32_t val = (a >> shift) & mask[i];
                ret = (val < mask[i] && val < ret) ? val : ret;
                printf("%u ", ret);
            }
            printf("\n");
        }
    }

    void clear()
    {
        for (int i = 0; i < d; ++i)
            delete[] A[i];
    }

    virtual uint32_t insert(const uint8_t *key, uint16_t key_len)
    {
        for (int i = 0; i < d; ++i)
            idx[i] = MurmurHash3_x86_32(key, key_len, hashseed[i]) % w[i];

        uint32_t ret = UINT32_MAX;
        for (int i = 0; i < d; ++i)
        {
            uint32_t &a = A[i][idx[i] >> cpw[i]];
            uint32_t shift = (idx[i] & lo[i]) << cs[i];
            uint32_t val = (a >> shift) & mask[i];
            a += (val < mask[i]) ? (1 << shift) : 0;
            ret = (val < mask[i] && val < ret) ? val : ret;
        }
        return ret + 1;
    }

    virtual uint32_t erase(const uint8_t *key, uint16_t key_len)
    {

        for (int i = 0; i < d; ++i)
            idx[i] = MurmurHash3_x86_32(key, key_len, hashseed[i]) % w[i];

        uint32_t ret = UINT32_MAX;
        for (int i = 0; i < d; ++i)
        {
            uint32_t &a = A[i][idx[i] >> cpw[i]];
            uint32_t shift = (idx[i] & lo[i]) << cs[i];
            uint32_t val = (a >> shift) & mask[i];
            a -= (val < mask[i] && val > 0) ? (1 << shift) : 0;
            ret = (val < mask[i] && val < ret && val > 0) ? val : ret;
        }
        return ret - 1;
    }

    uint32_t query(const uint8_t *key, uint16_t key_len)
    {
        uint32_t ret = UINT32_MAX;
        for (int i = 0; i < d; ++i)
        {
            uint32_t idx = MurmurHash3_x86_32(key, key_len, hashseed[i]) % w[i];
            uint32_t a = A[i][idx >> cpw[i]];
            uint32_t shift = (idx & lo[i]) << cs[i];
            uint32_t val = (a >> shift) & mask[i];
            ret = (val < mask[i] && val < ret) ? val : ret;
        }
        return ret;
    }
};

class TowerSketchCU : public TowerSketch
{
public:
    TowerSketchCU() {}
    TowerSketchCU(uint32_t w_d) { init(w_d); }
    ~TowerSketchCU() {}

    uint32_t insert(const uint8_t *key, uint16_t key_len)
    {
        uint32_t min_val = UINT32_MAX;
        for (int i = 0; i < d; ++i)
            idx[i] = MurmurHash3_x86_32(key, key_len, hashseed[i]) % w[i];
        for (int i = 0; i < d; ++i)
        {
            uint32_t a = A[i][idx[i] >> cpw[i]];
            uint32_t shift = (idx[i] & lo[i]) << cs[i];
            uint32_t val = (a >> shift) & mask[i];
            min_val = (val < mask[i] && val < min_val) ? val : min_val;
        }
        for (int i = 0; i < d; ++i)
        {
            uint32_t &a = A[i][idx[i] >> cpw[i]];
            uint32_t shift = (idx[i] & lo[i]) << cs[i];
            uint32_t val = (a >> shift) & mask[i];
            a += (val < mask[i] && val == min_val) ? (1 << shift) : 0;
        }
        return min_val + 1;
    }
};

class TowerSketchHalfCU : public TowerSketch
{
public:
    TowerSketchHalfCU() {}
    TowerSketchHalfCU(uint32_t w_d) { init(w_d); }
    ~TowerSketchHalfCU() {}

    uint32_t insert(const uint8_t *key, uint16_t key_len)
    {
        uint32_t min_val = UINT32_MAX;
        for (int i = 0; i < d; ++i)
            idx[i] = MurmurHash3_x86_32(key, key_len, hashseed[i]) % w[i];
        for (int i = 0; i < d; ++i)
        {
            uint32_t &a = A[i][idx[i] >> cpw[i]];
            uint32_t shift = (idx[i] & lo[i]) << cs[i];
            uint32_t val = (a >> shift) & mask[i];
            if (val < mask[i] && val <= min_val)
            {
                a += 1 << shift;
                min_val = val;
            }
        }
        return min_val + 1;
    }
};
}
#endif