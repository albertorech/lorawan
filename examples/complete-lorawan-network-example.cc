/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lora-mac.h"
#include "ns3/gateway-lora-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/hex-grid-position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-status.h"
#include "ns3/end-device-status.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include <algorithm>
#include <ctime>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 2000;
int gatewayRings = 4;
int nGateways = 3 * gatewayRings * gatewayRings - 3 * gatewayRings + 1; //numero di gateway disposti esagonalmente a anelli
double radius = 15000; //raggio dell'area se aumento posso avere undersensitivity
double simulationTime = 600;
int appPeriodSeconds = 600;
std::vector<int> sfQuantity (6);

// Output control
bool printEDs = true;
bool buildingsEnabled = true;

void
PrintEndDevices (NodeContainer endDevices, NodeContainer gateways, std::string edFilename, std::string gwFilename)
{
  const char * c = edFilename.c_str ();
  std::ofstream edFile;
  edFile.open (c);
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLoraMac> ();
      int sf = int(mac->GetDataRate ());
      Vector pos = position->GetPosition ();
      edFile << pos.x << " " << pos.y << " " << sf << std::endl;
    }
  edFile.close ();
  // Also print the gateways
  c = gwFilename.c_str ();
  std::ofstream gwFile;
  gwFile.open (c);
  for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      Vector pos = position->GetPosition ();
      gwFile << pos.x << " " << pos.y << " GW" << std::endl;
    }
  gwFile.close ();
}
// stampa file di una colonna di SF visti dal network server
void
PrintSF (std::map<LoraDeviceAddress, Ptr<EndDeviceStatus>> endDeviceStatuses, std::string SFFilename)
{
  const char * c = SFFilename.c_str ();
  std::ofstream SFFile;
  SFFile.open (c);
  for (std::map<LoraDeviceAddress, Ptr<EndDeviceStatus>>::iterator i = endDeviceStatuses.begin (); i != endDeviceStatuses.end (); i++)
    {
      Ptr<EndDeviceStatus> EDStatus = (*i).second;
      ns3::EndDeviceStatus::ReceivedPacketList pktList = (*EDStatus).GetReceivedPacketList();
      for (ns3::EndDeviceStatus::ReceivedPacketList::iterator j = pktList.begin (); j != pktList.end (); j++)
        {
          ns3::EndDeviceStatus::ReceivedPacketInfo info = (*j).second;
          SFFile << (unsigned)info.sf << std::endl;
        }
    }
  SFFile.close ();
}

int main (int argc, char *argv[])
{

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("gatewayRings", "Number of gateway rings to include", gatewayRings);
  cmd.AddValue ("radius", "The radius of the area to simulate", radius);
  cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  cmd.AddValue ("appPeriod", "The period in seconds to be used by periodically transmitting applications", appPeriodSeconds);
  cmd.AddValue ("printEDs", "Whether or not to print a file containing the ED's positions", printEDs);

  cmd.Parse (argc, argv);

  double gatewayRadius = radius / ((gatewayRings - 1) * 2 + 1);

  // Set up logging
  LogComponentEnable ("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
  // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLoraMac", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraMacHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);

  /***********
   *  Setup  *
   ***********/

  // Compute the number of gateways
  nGateways = 3 * gatewayRings * gatewayRings - 3 * gatewayRings + 1;

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (0.0),
                                 "Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);

  if (buildingsEnabled)
    {
      //Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing = CreateObject<CorrelatedShadowingPropagationLossModel> ();

      //Aggregate shadowing to the logdistance loss
      loss->SetNext(shadowing);

      //Add the effect to the channel propagation loss
      Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      shadowing->SetNext(buildingLoss);
    }


  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LoraMacHelper
  LoraMacHelper macHelper = LoraMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes, creo nDevices endDevices
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to each node
  mobility.Install (endDevices);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j)
        {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      position.z = 1.2;
      mobility->SetPosition (position);
    }

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LoraMacHelper::ED);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  // Connect trace sources
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
    }

  /*********************
   *  Create Gateways  *
   *********************/

  // Create the gateway nodes (allocate them uniformely on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  // Make it so that nodes are at a certain height > 0
  allocator->Add (Vector (0.0, 0.0, 15.0));

  //create hex-grid
  Ptr<HexGridPositionAllocator> grid = CreateObject<HexGridPositionAllocator> (gatewayRadius);
  mobility.SetPositionAllocator (grid);

  mobility.Install (gateways);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = gateways.Begin ();
       j != gateways.End (); ++j)
    {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      position.z = 15;
      mobility->SetPosition (position);
    }

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LoraMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  /**********************
   *  Handle buildings  *
   **********************/

  double xLength = 130;
  double deltaX = 32;
  double yLength = 64;
  double deltaY = 17;
  int gridWidth = 2*radius/(xLength+deltaX);
  int gridHeight = 2*radius/(yLength+deltaY);
  if (buildingsEnabled == false)
    {
      gridWidth = 0;
      gridHeight = 0;
    }
  Ptr<GridBuildingAllocator> gridBuildingAllocator;
  gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
  gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
  gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
  gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
  gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
  gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
  gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
  gridBuildingAllocator->SetAttribute ("MinX", DoubleValue (-gridWidth*(xLength+deltaX)/2+deltaX/2));
  gridBuildingAllocator->SetAttribute ("MinY", DoubleValue (-gridHeight*(yLength+deltaY)/2+deltaY/2));
  BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

  BuildingsHelper::Install (endDevices);
  BuildingsHelper::Install (gateways);
  BuildingsHelper::MakeMobilityModelConsistent ();

  // Print the buildings
  std::ofstream myfile;
  myfile.open ("buildings.txt");
  std::vector<Ptr<Building> >::const_iterator it;
  int j = 1;
  for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j)
    {
      Box boundaries = (*it)->GetBoundaries ();
      myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
    }
  myfile.close();




  /**********************************************
   *  Set up the end device's spreading factor  *
   **********************************************/

  sfQuantity = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  ////////////
  // Create NS
  ////////////

  NodeContainer networkServers;
  networkServers.Create (1);

  // Install the SimpleNetworkServer application on the network server

  NetworkServerHelper networkServerHelper;
  networkServerHelper.SetGateways (gateways);
  networkServerHelper.SetEndDevices (endDevices);
  ApplicationContainer nsAppContainer = networkServerHelper.Install (networkServers);



  Ptr<NetworkServer> ns = nsAppContainer.Get(0)->GetObject<NetworkServer> ();
  Ptr<NetworkStatus> status = ns->GetNetworkStatus ();

  // Install the Forwarder application on the gateways

  ForwarderHelper forwarderHelper;
  forwarderHelper.Install (gateways);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  Time appStopTime = Seconds (simulationTime);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  appHelper.SetPacketSize (23);
  Ptr <RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> ("Min", DoubleValue (0), "Max", DoubleValue (10));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /**********************
   * Print output files *
   *********************/

  if (printEDs)
    {
      PrintEndDevices (endDevices, gateways,
                       "nodeLocations.dat", "gwLocations.dat");
    }

  /****************
   *  Simulation  *
   ****************/

  Simulator::Stop (appStopTime + Hours (2));

  // PrintSimulationTime ();

  Simulator::Run ();

  Simulator::Destroy ();


  std::map<LoraDeviceAddress, Ptr<EndDeviceStatus> > endDeviceStatuses = status->m_endDeviceStatuses;
  //std::map<Address, Ptr<GatewayStatus>> gatewayStatuses= status->m_gatewayStatuses;
  PrintSF(endDeviceStatuses, "printSF.dat");
  double received = 0;
  std::string PowFilename = "potenze.dat";
  std::string NumGwPerPktFilename = "gwperpkt.dat";
  const char * c = PowFilename.c_str ();
  const char * d = NumGwPerPktFilename.c_str ();
  std::ofstream PowFile;
  std::ofstream NumGwPerPktFile;
  PowFile.open (c);
  NumGwPerPktFile.open (d);
  for (std::map<LoraDeviceAddress, Ptr<EndDeviceStatus>>::iterator i = endDeviceStatuses.begin (); i != endDeviceStatuses.end (); i++)
    {
      Ptr<EndDeviceStatus> EDStatus = (*i).second;
      ns3::EndDeviceStatus::ReceivedPacketList pktList = (*EDStatus).GetReceivedPacketList();

      received += (double)pktList.size();

      for (ns3::EndDeviceStatus::ReceivedPacketList::iterator j = pktList.begin (); j != pktList.end (); j++)
        {
          ns3::EndDeviceStatus::ReceivedPacketInfo info = (*j).second;
          ns3::EndDeviceStatus::GatewayList gatewayList = info.gwList;
          Ptr<Packet const> pkt=(*j).first;
          NumGwPerPktFile << pkt <<" "<<gatewayList.size()<< std::endl;
          for (ns3::EndDeviceStatus::GatewayList::iterator k = gatewayList.begin (); k != gatewayList.end (); k++)
            {
              ns3::EndDeviceStatus::PacketInfoPerGw infoPerGw = (*k).second;
              PowFile << infoPerGw.gwAddress<<" "<<infoPerGw.rxPower << std::endl;
            }
        }
    }
  PowFile.close();
  NumGwPerPktFile.close();
  std::cout<<received<<"\n";
  double receivedProb = received/nDevices;
  std::cout<<receivedProb<<"\n";
  return 0;
}
