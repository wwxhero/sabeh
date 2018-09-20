#include "action.h"


CSetEnvCondition::CSetEnvCondition( const CActionParseBlock* pBlock, CHcsmCollection* pColl)
{
	assert(pBlock);
	m_pHC = pColl;
    m_setting = pBlock->GetVarName(); //<turn on/off action;
    m_value = pBlock->GetVarValue();	
    m_isVar = pBlock->IsCellTypeVariable();
    InitCandidateSet( pBlock );
}
CSetEnvCondition& 
CSetEnvCondition::operator=( const CSetEnvCondition& cRhs ){
	if (this != &cRhs){
        m_pHC = cRhs.m_pHC;
	}
	return *this;
}
CSetEnvCondition::CSetEnvCondition(const CSetEnvCondition&  cRhs){
	*this =  cRhs;
}
////////////////////////////////////////////////////////////////////////
///\remark
///		copies headlight command to m_sHeadlightScenarioControl
////////////////////////////////////////////////////////////////////////
void CSetEnvCondition::Execute( const set<CCandidate>* cpInstigators ){
	float value = 0.0;
    if (m_isVar){
        value = CHcsmCollection::GetExprVariable(m_value);
    }else{
        value = std::stof(m_value);
    }
    CHcsmCollection::THeadlightSetting setting = m_pHC->GetHeadLightSettings();

    if (m_setting =="Headlight.On" )                    { setting.On                = value>0;}
    else if (m_setting =="Headlight.Azimuth")           { setting.Azimuth           = value;}
    else if (m_setting =="Headlight.Elevation" )        { setting.Elevation         = value;}
    else if (m_setting =="Headlight.BeamWidth" )        { setting.BeamWidth         = value;}
    else if (m_setting =="Headlight.BeamHeight" )       { setting.BeamHeight        = value;}
    else if (m_setting =="Headlight.ConstCoeff" )       { setting.ConstCoeff        = value;}
    else if (m_setting =="Headlight.LinearCoeff" )      { setting.LinearCoeff       = value;}
    else if (m_setting =="Headlight.QuadCoeff" )        { setting.QuadCoeff         = value;}
    else if (m_setting =="Headlight.HeightFade" )       { setting.HeightFade        = value;}
    else if (m_setting =="Headlight.Intensity" )        { setting.Intensity         = value;}
    else if (m_setting =="Headlight.CutOffDist" )       { setting.CutOffDist        = value;}
    else if (m_setting =="Headlight.LampSeparation" )   { setting.LampSeparation    = value;}
    else if (m_setting =="Headlight.LampForwardOffset" ){ setting.LampForwardOffset = value;}
    
    m_pHC->SetHeadLightSettings(setting);
}

CSetEnvCondition::~CSetEnvCondition(void)
{
}