//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef BEACONING_PROCESSOR_PROCESSOR_H_
#define BEACONING_PROCESSOR_PROCESSOR_H_

#include <omnetpp.h>
#include <map>
#include <list>
#include <inet/common/InitStages.h>
#include <inet/linklayer/common/MacAddress.h>
#include <inet/common/geometry/common/Coord.h>
#include <inet/common/geometry/common/EulerAngles.h>
#include "../base/BeaconingBase.h"
#include "../base/Beacon_m.h"
#include "../neighbortable/PacketLossEstimator.h"
#include "../neighbortable/NeighborTable.h"
#include "../base/BeaconReport_m.h"
#include "../../mobility/DroneLinearMobility.h"
#include "../renewalbeaconing/RenewalBeaconing.h"
#include "LaneUtil.h"
#include "../../roadSystem/RoadSystem.h"

using namespace omnetpp;


struct AlgorithmOutput {
    Lane l;
    double speed;
};



class Processor : public cSimpleModule {

    protected:
        std::string mode = "simple";
        Lane laneSwitchTo;
        std::map<Lane, std::vector<MacAddress>> neighborNodes;
        std::map<Lane, std::vector<MacAddress>> nNeighborNodes;
        RoadSystem* rs = nullptr;
        std::string fisrtPart;
        std::string endPart;
        RoadPath rp;
        std::string currentPart;
        int currentI;
        int currentJ;
        cGate* gateToM = nullptr;
        cGate* gateMIn = nullptr;
        NeighborTable*  nbTable;
        std::map<NodeIdentifier, int> nbSeqTable;
        std::map<NodeIdentifier, int> nblostTable;

        int        gidBeaconsIn;
        NodeIdentifier  ownIdentifier;
        DroneLinearMobility *mobility;
        RenewalBeaconing *rb;
        cMessage  *startPlan;

        double preferredSpeed;
        double maxSpeed;
        double clTimeThreshold;
        double overtakeSpeed;
        double checkOvertakeSlowPeriod = 0.5;
        double minimumSafetyDistance;
        double maxNeighborRange;
        double otherLaneSafetyRange;
        double targetLaneSafetyRange;

        double totalSpeedCost = 0;
        double totalPositionCost = 0;
        double totalCost = 0;
        double totalSwitchCost = 0;


        double overtakePlan_arrivalTime = -1;

        bool couldMove = false;
        double waitTime = 0;
        simtime_t lastRun;

        double KAPPA_Speed;
        double KAPPA_Position;
        double KAPPA_Switch;


        double KAPPA_STOP = 2;  // equals to algorithm interval
        double KAPPA_SINTERVAL = 0.3;
        double E_SPEED = 2 * 15;  // algorithm * maximum speed
        double E_TARGET = 16;
        double E_NEIGHBOR = 30;
        const double MAX = 100000;
    public:
        //------------------------------------------------------------------------------------------------------------------------------------------------------------
        //---------------------                   Basic Function                                                       --------------------
        //------------------------------------------------------------------------------------------------------------------------------------------------------------
        virtual ~Processor();
        Processor();
        virtual void finish() override;
        void initialize(int stage);
        virtual int  numInitStages () const override { return inet::NUM_INIT_STAGES; };
        void handleMessage(cMessage* msg);
        void moveToNextPart();
        void setNextNextTarget();
        int getCurrentI() {return currentI;};
        int getCurrentJ() {return currentJ;};
        std::string getCurrentRoadId(){return currentPart;};
        int getPacketLoss();
        void recordBeacon (BeaconReport* beacon);
        NeighborTable  getnbTable();
        void printTable();
        void updateClTime(MacAddress nb);
        void handleMovingInfo(MovingInfo* mi);
        //------------------------------------------------------------------------------------------------------------------------------------------------------------
        //---------------------                    Lane Switching Algorithm Part                                                                  --------------------
        //------------------------------------------------------------------------------------------------------------------------------------------------------------

        void makeDecision(AlgorithmOutput output);

        /***
         * finding the next target lane
         */
        AlgorithmOutput findNextLane();

        /***
         * Calculate the speed cost of the given lane l, if there is no ahead drone return 0
         */
        double getSpeedCost(Lane l);

        /***
         * Update the candidate nodes list, include neighbor drones and drones in neighbor's neighbor lane.
         */
        void getCandidateNodes();

        /***
         * Clear the candidate nodes list after running algorithm
         */
        void clearCandidateNodes();

        /***
         * Calculate the potential cost of the given lane l, if the cost != MAX and the current drone is the ahead-est one, modified it to 0
         */
        double getCollisionCost(Lane l);

        /***
         * Calculate the potential cost of the given lane l, if the cost != MAX and the current drone is the ahead-est one, modified it to 0
         */
        double getPositionCost(Lane l);

        void runAlgorithm();

//------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------

};

#endif /* BEACONING_PROCESSOR_PROCESSOR_H_ */
