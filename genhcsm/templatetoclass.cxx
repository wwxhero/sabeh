#include "genhcsm.h"
#include "hcsmcollection.h"

CHcsm* CHcsmCollection::GetClassFromTemplateName(
            string templateName,
            const CSnoBlock& snoBlock
            )
{
    CHcsm* pHcsm;

    if ( templateName == "Ddo" ) {

        // EXIT: found class for template
        pHcsm = new CDdo( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "VirtualObject" ) {

        // EXIT: found class for template
        pHcsm = new CVirtualObject( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "TimeTrigger" ) {

        // EXIT: found class for template
        pHcsm = new CTimeTrigger( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "GmtrcPstnTrigger" ) {

        // EXIT: found class for template
        pHcsm = new CGmtrcPstnTrigger( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "TrffcLghtTrigger" ) {

        // EXIT: found class for template
        pHcsm = new CTrffcLghtTrigger( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "RoadPadTrigger" ) {

        // EXIT: found class for template
        pHcsm = new CRoadPadTrigger( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "ExpressionTrigger" ) {

        // EXIT: found class for template
        pHcsm = new CExpressionTrigger( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "FollowTrigger" ) {

        // EXIT: found class for template
        pHcsm = new CFollowTrigger( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "TimeToArrvlTrigger" ) {

        // EXIT: found class for template
        pHcsm = new CTimeToArrvlTrigger( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "Gateway" ) {

        // EXIT: found class for template
        pHcsm = new CGateway( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "Ado" ) {

        // EXIT: found class for template
        pHcsm = new CAdo( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "Autonomous" ) {

        // EXIT: found class for template
        pHcsm = new CAutonomous( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "RemoteControl" ) {

        // EXIT: found class for template
        pHcsm = new CRemoteControl( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "FreeDrive" ) {

        // EXIT: found class for template
        pHcsm = new CFreeDrive( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "Follow" ) {

        // EXIT: found class for template
        pHcsm = new CFollow( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "LaneChange" ) {

        // EXIT: found class for template
        pHcsm = new CLaneChange( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "LcMonitor" ) {

        // EXIT: found class for template
        pHcsm = new CLcMonitor( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "LcSignal" ) {

        // EXIT: found class for template
        pHcsm = new CLcSignal( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "LcExecute" ) {

        // EXIT: found class for template
        pHcsm = new CLcExecute( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "LcAbort" ) {

        // EXIT: found class for template
        pHcsm = new CLcAbort( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "LcExecuteNeutralize" ) {

        // EXIT: found class for template
        pHcsm = new CLcExecuteNeutralize( this );
        return pHcsm;

    }
    if ( templateName == "LcExecuteIncrement" ) {

        // EXIT: found class for template
        pHcsm = new CLcExecuteIncrement( this );
        return pHcsm;

    }
    if ( templateName == "LcExecuteSteady" ) {

        // EXIT: found class for template
        pHcsm = new CLcExecuteSteady( this );
        return pHcsm;

    }
    if ( templateName == "LcExecuteDecrement" ) {

        // EXIT: found class for template
        pHcsm = new CLcExecuteDecrement( this );
        return pHcsm;

    }
    if ( templateName == "NavigateIntrsctn" ) {

        // EXIT: found class for template
        pHcsm = new CNavigateIntrsctn( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "Merge" ) {

        // EXIT: found class for template
        pHcsm = new CMerge( this );
        return pHcsm;

    }
    if ( templateName == "EnvironmentController" ) {

        // EXIT: found class for template
        pHcsm = new CEnvironmentController( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "EnviroInfo" ) {

        // EXIT: found class for template
        pHcsm = new CEnviroInfo( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "IntersectionManager" ) {

        // EXIT: found class for template
        pHcsm = new CIntersectionManager( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "OwnVehicleMirror" ) {

        // EXIT: found class for template
        pHcsm = new COwnVehicleMirror( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "StaticObjManager" ) {

        // EXIT: found class for template
        pHcsm = new CStaticObjManager( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "TrafficLightManager" ) {

        // EXIT: found class for template
        pHcsm = new CTrafficLightManager( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "TrafficManager" ) {

        // EXIT: found class for template
        pHcsm = new CTrafficManager( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "TrafficSource" ) {

        // EXIT: found class for template
        pHcsm = new CTrafficSource( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "VehFail" ) {

        // EXIT: found class for template
        pHcsm = new CVehFail( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "DriverMirror" ) {

        // EXIT: found class for template
        pHcsm = new CDriverMirror( this );
        return pHcsm;

    }
    if ( templateName == "TrafficData" ) {

        // EXIT: found class for template
        pHcsm = new CTrafficData( this, snoBlock );
        return pHcsm;

    }
    if ( templateName == "DaqDriver" ) {

        // EXIT: found class for template
        pHcsm = new CDaqDriver( this, snoBlock );
        return pHcsm;

    }

    return NULL;
}
