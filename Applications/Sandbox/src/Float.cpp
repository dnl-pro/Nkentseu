#include "Float.h"
#include "NKLogger/NkLog.h"
#include "NKContainers/String/NkStringUtils.h"
#include <bitset>
#include <vector>
#include <cstdint>
#include <cstring>

using namespace nkentseu::string;

float NkMath::kahanSum(std::vector<double>& data){
    float s=0.0f,c=0.0f;
    for(int i=0;i<data.size();++i){float y=data[i]-c;float t=s+y;c=(t-s)-y;s=t;}
    return s;
}

float NkMath::kahanSum(std::vector<float>& data){
    float s=0.0f,c=0.0f;
    for(int i=0;i<data.size();++i){float y=data[i]-c;float t=s+y;c=(t-s)-y;s=t;}
    return s;
}

void NkMath::inspectFloat(float x){
    uint32_t b;std::memcpy(&b,&x,sizeof(b));
    uint32_t s=(b>>31)&1,e=(b>>23)&0xFF,m=b&0x7FFFFF;
    logger.Info("Float {0} binaire:\nSigne:{1}\nExposant:{2}\nMantisse:{3}",x,s,e,m);
}

void NkMath::inspectDouble(double x){
    uint64_t b;std::memcpy(&b,&x,sizeof(b));
    uint64_t s=(b>>63)&1,e=(b>>52)&0x7FF,m=b&0xFFFFFFFFFFFFF;
    logger.Info("Double {0} binaire:\nSigne:{1}\nExposant:{2}\nMantisse:{3}",x,s,e,m);
}

float NkMath::varianceNaive(const std::vector<float>& data){
    float sum=0.0f,sumSq=0.0f;
    for(float x:data){sum+=x;sumSq+=x*x;}
    float m=sum/data.size();return (sumSq/data.size())-(m*m);
}

float NkMath::varianceWelford(const std::vector<float>& data){
    float mean=0.0f,M2=0.0f;int n=0;
    for(float x:data){++n;float d=x-mean;mean+=d/n;M2+=d*(x-mean);}
    return M2/n;
}

float NkMath::epsilonMachine(){
    float eps=1.0f;
    while((1.0f+eps/2.0f)>1.0f)eps/=2.0f;
    return eps;
}