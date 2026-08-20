import pyflamegpu
import sys
from datetime import datetime


emit_messages_walls = r"""
FLAMEGPU_AGENT_FUNCTION(emit_messages_walls, flamegpu::MessageNone, flamegpu::MessageSpatial2D) {
  // Generate messages with current location of walls 

  const float x = FLAMEGPU->getVariable<float>("x");
  const float y = FLAMEGPU->getVariable<float>("y");
  const float m = FLAMEGPU->getVariable<float>("m");
  const float n = FLAMEGPU->getVariable<float>("n");
  const float p = FLAMEGPU->getVariable<float>("p");
  
  FLAMEGPU->message_out.setVariable<float>("x", x); 
  FLAMEGPU->message_out.setVariable<float>("y", y);
  FLAMEGPU->message_out.setVariable<float>("m", m);
  FLAMEGPU->message_out.setVariable<float>("n", n);
  FLAMEGPU->message_out.setVariable<float>("p", p);
  FLAMEGPU->message_out.setVariable<int>("type", 0);
  FLAMEGPU->message_out.setVariable<flamegpu::id_t>("id", FLAMEGPU->getID());

  return flamegpu::ALIVE;
}
"""


emit_messages_people = r"""
FLAMEGPU_AGENT_FUNCTION(emit_messages_people, flamegpu::MessageNone, flamegpu::MessageSpatial2D) {
  // Generate messages with current location of people 

  const float x = FLAMEGPU->getVariable<float>("x");
  const float y = FLAMEGPU->getVariable<float>("y");
  const float m = FLAMEGPU->getVariable<float>("m");
  const float n = FLAMEGPU->getVariable<float>("n");
  const float p = FLAMEGPU->getVariable<float>("p");
  
  FLAMEGPU->message_out.setVariable<float>("x", x); 
  FLAMEGPU->message_out.setVariable<float>("y", y);
  FLAMEGPU->message_out.setVariable<float>("m", m);
  FLAMEGPU->message_out.setVariable<float>("n", n);
  FLAMEGPU->message_out.setVariable<float>("p", p);
  FLAMEGPU->message_out.setVariable<int>("type", 1);
  FLAMEGPU->message_out.setVariable<flamegpu::id_t>("id", FLAMEGPU->getID());
  
  return flamegpu::ALIVE;
}
"""


check_field = r"""
FLAMEGPU_AGENT_FUNCTION(check_field, flamegpu::MessageSpatial2D, flamegpu::MessageNone) {
  float x = FLAMEGPU->getVariable<float>("x");
  float y = FLAMEGPU->getVariable<float>("y");
  float cap = FLAMEGPU->getVariable<float>("cap");
  int observation_type = FLAMEGPU->environment.getProperty<int>("checkfield_type");
  float totalfield = 0;

  //Set when to gather data.
  if(FLAMEGPU->getStepCounter() != FLAMEGPU->environment.getProperty<int>("checkfield_cycles")){
    //We only need this function to run once
    return flamegpu::ALIVE;
  }

  //Calculate the sum of the field effect from surrounding objects 
  for (const auto &message : FLAMEGPU->message_in(x, y)) {
         
    float obs_x = message.getVariable<float>("x");
    float obs_y = message.getVariable<float>("y");
    float obs_m = message.getVariable<float>("m");
    float obs_n = message.getVariable<float>("n");
    float obs_p = message.getVariable<float>("p");
    int obs_type = message.getVariable<int>("type"); // 0 - wall. 1 - person.
    
    float field = obs_m/powf((obs_n*sqrt((x-obs_x)*(x-obs_x)+(y-obs_y)*(y-obs_y))),obs_p);

    if(observation_type == 0){
      // Combined field of walls and people (replicates move_people)
      if(obs_type == 0 && (field > cap))
	totalfield += cap;
      else
	totalfield += field;
      
    }else if(observation_type == 1){
      //Look at field due to walls only
      if(obs_type == 0){
	if(field > cap)
	  totalfield += cap;
	else
	  totalfield += field;
      }
      
    } else {
      //Look at field due to people only
      if(obs_type == 1){
	totalfield += field;
      }
    }  
  }
  
  FLAMEGPU->setVariable<float>("totalfield", totalfield);
  
  return flamegpu::ALIVE;
}
"""

move_people = r"""
FLAMEGPU_AGENT_FUNCTION(move_people, flamegpu::MessageSpatial2D, flamegpu::MessageNone) {
  
  float timestep = FLAMEGPU->environment.getProperty<float>("timestep");
  float weighting = FLAMEGPU->environment.getProperty<float>("weighting");
  float lookahead = FLAMEGPU->environment.getProperty<float>("lookahead_ped");
  float waypoint_accuracy_ped = FLAMEGPU->environment.getProperty<float>("waypoint_accuracy_ped");
  float x = FLAMEGPU->getVariable<float>("x");
  float y = FLAMEGPU->getVariable<float>("y");
  int journeyIndex = FLAMEGPU->getVariable<int>("journeyIndex");
  float destx = FLAMEGPU->getVariable<float, 10>("destx", journeyIndex);
  float desty = FLAMEGPU->getVariable<float, 10>("desty", journeyIndex);
  int dwelltime = FLAMEGPU->getVariable<int>("dwelltime");
  float velx = FLAMEGPU->getVariable<float>("velx");
  float vely = FLAMEGPU->getVariable<float>("vely");
  float steering = FLAMEGPU->getVariable<float>("steering");
  float maxSpeed = FLAMEGPU->getVariable<float>("maxSpeed");
  float minSep = FLAMEGPU->getVariable<float>("minSep");
  float cap = FLAMEGPU->getVariable<float>("cap");
  const flamegpu::id_t ID = FLAMEGPU->getID();
  float distRemaining = 0;
  float newdistRemaining = 0;
  float distReduction = 0;
  float score1, score2, score3, finalscore;
  float dx,dy,separation;
  float vel,theta,newtheta;
  float newx,newy;

  // Overall basis: consider three directions for next move, and take the best.
  // Directions tested are (1) straight ahead, (2) turn clockwise by steering angle, (3) turn anti-clockwise by steering angle

  //If someone is pausing at a waypoint they don't need to move or steer ///////////////////////////////////////////////////////////
  if(dwelltime > 0){
    //Decrement the counter but return without moving the person
    dwelltime--;
    FLAMEGPU->setVariable<int>("dwelltime", dwelltime);
    return flamegpu::ALIVE;
  }

  //A person approaching a waypoint (but not their final destination) will select a new target direction and might pause ///////////
  dx = destx - x;
  dy = desty - y;
  distRemaining = sqrt(dx*dx + dy*dy);
  
  if(journeyIndex > 0 && distRemaining < waypoint_accuracy_ped){
    //Agent is within range of a waypoint. They might pause here, set dwelltime countdown prior to updating the journey index
    dwelltime = FLAMEGPU->getVariable<int, 10>("dwell", journeyIndex);
    
    journeyIndex--;
    FLAMEGPU->setVariable<int>("journeyIndex", journeyIndex);
    
    if(dwelltime == 0){
      //No need to pause, update with new destination and continue towards it
      destx = FLAMEGPU->getVariable<float, 10>("destx", journeyIndex);
      desty = FLAMEGPU->getVariable<float, 10>("desty", journeyIndex);
      dx = destx - x;
      dy = desty - y;
      distRemaining = sqrt(dx*dx + dy*dy);
    }else{
      //Set the dwell countdown to begin pausing at the waypoint - no need for further action this timestep
      FLAMEGPU->setVariable<int>("dwelltime", dwelltime);
      return flamegpu::ALIVE;
    }
  }

  //Move directly ahead (based on current velocity)//////////////////////////////////////////////////////////////////////////////////
  //Slow down if the agent is very near the target location. This carries through to the cases below
  if(distRemaining < 5)
    lookahead = lookahead * 0.2;
  
  newx = x + (velx * lookahead); 
  newy = y + (vely * lookahead);
  score1 = 0;

  for (const auto &message : FLAMEGPU->message_in(newx, newy)) {
    //Scan across all the messages around our point of interest - low score is a good direction to move.
    //Calculate the sum of the field effect from surrounding objects
    //Must include both fixed (walls) and mobile (people) obstructions
    
    if (message.getVariable<flamegpu::id_t>("id") != ID) {
      
      float obs_x = message.getVariable<float>("x");
      float obs_y = message.getVariable<float>("y");
      float obs_m = message.getVariable<float>("m");
      float obs_n = message.getVariable<float>("n");
      float obs_p = message.getVariable<float>("p");
      int obs_type = message.getVariable<int>("type"); // 0 - wall. 1 - person.
      
      float field = obs_m/powf((obs_n*sqrt((newx-obs_x)*(newx-obs_x)+(newy-obs_y)*(newy-obs_y))),obs_p);
           
      if(obs_type == 0 && (field > cap))
	score1 += cap;
      else
	score1 += field;

      //Record the minimum separation of this agent from other people during the run
      //Could be adapted to look at proximity to walls
      separation = sqrt((x-obs_x)*(x-obs_x)+(y-obs_y)*(y-obs_y));
      if(obs_type == 1 && separation < minSep)
	minSep = separation; 
    }
  }
  FLAMEGPU->setVariable<float>("minSep", minSep);

  //Find distance to destination (or next waypoint)
  dx = destx - newx;
  dy = desty - newy;
  newdistRemaining = sqrt(dx*dx + dy*dy);
  distReduction = distRemaining - newdistRemaining; // Lots of potential for problems here with negative numbers

  //Weighted total combined with field score
  //Note - distReduction, high is good, so subtract it from the obstacle score to retain 'low is good' basis 
  score1 -= distReduction * weighting;
  
  //printf("x: %f y: %f newx: %f newy: %f dx: %f dy: %f score: %f distReduction: %f \n", x, y, newx, newy, dx, dy, score1, -1.0*distReduction);

  //Turn 10deg clockwise/////////////////////////////////////////////////////////////////////////////////////////////////////// 
  theta = atan2(vely, velx);
  vel = sqrt((velx*velx) + (vely*vely));
  newtheta = theta - steering;
  velx = vel*cos(newtheta);
  vely = vel*sin(newtheta);
  newx = x + (velx * lookahead); 
  newy = y + (vely * lookahead);
  score2 = 0;
  for (const auto &message : FLAMEGPU->message_in(newx, newy)) {
    
    if (message.getVariable<flamegpu::id_t>("id") != ID) {
      
      float obs_x = message.getVariable<float>("x");
      float obs_y = message.getVariable<float>("y");
      float obs_m = message.getVariable<float>("m");
      float obs_n = message.getVariable<float>("n");
      float obs_p = message.getVariable<float>("p");
      int obs_type = message.getVariable<int>("type"); // 0 - wall. 1 - person.
      
      float field = obs_m/powf((obs_n*sqrt((newx-obs_x)*(newx-obs_x)+(newy-obs_y)*(newy-obs_y))),obs_p);
           
      if(obs_type == 0 && (field > cap))
	score2 += cap;
      else
	score2 += field;
    }
  }
 
  //Find distance to destination (or next waypoint)
  dx = destx - newx;
  dy = desty - newy;
  newdistRemaining = sqrt(dx*dx + dy*dy);
  distReduction = distRemaining - newdistRemaining; 
  score2 -= distReduction * weighting;
  
  //Turn 10deg anti-clockwise///////////////////////////////////////////////////////////////////////////////////////////////////////
  newtheta = theta + steering;
  velx = vel*cos(newtheta);
  vely = vel*sin(newtheta);
  newx = x + (velx * lookahead); 
  newy = y + (vely * lookahead);
  score3 = 0;
  for (const auto &message : FLAMEGPU->message_in(newx, newy)) {
    
    if (message.getVariable<flamegpu::id_t>("id") != ID) {
      
      float obs_x = message.getVariable<float>("x");
      float obs_y = message.getVariable<float>("y");
      float obs_m = message.getVariable<float>("m");
      float obs_n = message.getVariable<float>("n");
      float obs_p = message.getVariable<float>("p");
      int obs_type = message.getVariable<int>("type"); // 0 - wall. 1 - person.
      
      float field = obs_m/powf((obs_n*sqrt((newx-obs_x)*(newx-obs_x)+(newy-obs_y)*(newy-obs_y))),obs_p);
           
      if(obs_type == 0 && (field > cap))
	score3 += cap;
      else
	score3 += field;
    }
  }
 
  //Find distance to destination (or next waypoint)
  dx = destx - newx;
  dy = desty - newy;
  newdistRemaining = sqrt(dx*dx + dy*dy);
  distReduction = distRemaining - newdistRemaining; 
  score3 -= distReduction * weighting;

  //Duplicted calculations here - can be streamlined

  // Assume score 1 (straight on) is lowest until shown otherwise
  finalscore = score1;
  velx = maxSpeed*cos(theta);
  vely = maxSpeed*sin(theta);
  newx = x + (velx * timestep); // Might need to look further ahead than this to see meaningful change in field?
  newy = y + (vely * timestep);
  
  if(score2 < finalscore){
    finalscore = score2;

    newtheta = theta - steering;
    velx = maxSpeed*cos(newtheta);
    vely = maxSpeed*sin(newtheta);
    newx = x + (velx * timestep); 
    newy = y + (vely * timestep);
  }

  if(score3 < finalscore){
    finalscore = score3;

    newtheta = theta + steering;
    velx = maxSpeed*cos(newtheta);
    vely = maxSpeed*sin(newtheta);
    newx = x + (velx * timestep); 
    newy = y + (vely * timestep);
  }

  //Store new location
  FLAMEGPU->setVariable<float>("x", newx); 
  FLAMEGPU->setVariable<float>("y", newy);
  FLAMEGPU->setVariable<float>("velx", velx); 
  FLAMEGPU->setVariable<float>("vely", vely);
 
  return flamegpu::ALIVE;
}
"""


destination = r"""
FLAMEGPU_AGENT_FUNCTION_CONDITION(destination) {
  
  float x = FLAMEGPU->getVariable<float>("x");
  float y = FLAMEGPU->getVariable<float>("y");
  int journeyIndex = FLAMEGPU->getVariable<int>("journeyIndex");
  float destx = FLAMEGPU->getVariable<float, 10>("destx", journeyIndex);
  float desty = FLAMEGPU->getVariable<float, 10>("desty", journeyIndex);
  float distRemaining = 0;
  float dx,dy;
  float accuracy_ped = FLAMEGPU->environment.getProperty<float>("accuracy_ped");
  
  //Need something to set agent state to inactive once it is withing some 'error' level of the destination (i.e. remaining distance very small)
  dx = destx - x;
  dy = desty - y;
  distRemaining = sqrt(dx*dx + dy*dy);

  if(journeyIndex == 0 && distRemaining < accuracy_ped){
    //If they are very close to the final destination (not a waypoint) remove the agent from the similation
    //Level of accuracy could be made velocity dependent, to avoid faster agents going past the destination in their next timestep
    return true;
  }else{
    return false;
  }
}
"""


starttime = r"""
FLAMEGPU_AGENT_FUNCTION_CONDITION(starttime) {
  
  int starttime = FLAMEGPU->getVariable<int>("starttime");
  
  if(starttime == FLAMEGPU->getStepCounter()){
    return true;
  }else{
    return false;
  }
}
"""

start = r"""
FLAMEGPU_AGENT_FUNCTION(start, flamegpu::MessageNone, flamegpu::MessageNone) {
  return flamegpu::ALIVE;
}
"""

arrived = r"""
FLAMEGPU_AGENT_FUNCTION(arrived, flamegpu::MessageNone, flamegpu::MessageNone) {
  return flamegpu::ALIVE;
}
"""


class InitWalls(pyflamegpu.HostFunction):
    def run(self, FLAMEGPU):
        m_wall = FLAMEGPU.environment.getPropertyFloat("m_wall")
        n_wall = FLAMEGPU.environment.getPropertyFloat("n_wall")
        p_wall = FLAMEGPU.environment.getPropertyFloat("p_wall")
        # Get population of agents which will have been loaded from an external configuration file
        t_pop2 = FLAMEGPU.agent("wall")
        
        i = 0
        
        txt = f"Creating walls and obstacle agents\n"
        print(txt) 
        
        # Get DeviceAgentVector to the wall population
        t_vector = t_pop2.getPopulationData()
        AGENT_COUNT = len(t_vector)
        # Set defaults for all walls/obstacles
        for i in range(AGENT_COUNT):
            t = t_vector[i]
            t.setVariableFloat("m", m_wall)
            t.setVariableFloat("n", n_wall)
            t.setVariableFloat("p", p_wall)
            
        txt = f"Created {i} wall agents"
        print(txt) 
            


class InitPeople(pyflamegpu.HostFunction):
    def run(self, FLAMEGPU):
        m_ped = FLAMEGPU.environment.getPropertyFloat("m_ped")
        n_ped = FLAMEGPU.environment.getPropertyFloat("n_ped")
        p_ped = FLAMEGPU.environment.getPropertyFloat("p_ped")
        cap_ped = FLAMEGPU.environment.getPropertyFloat("cap_ped")
        # Minor random variation in steering angle to avoid lock-ups is introduced per agent
        steering_ped = FLAMEGPU.environment.getPropertyFloat("steering_ped")



        i = 0
        # count = 0

        txt = f"Creating people agents\n"
        print(txt) 

        # Include state "waiting" otherwise Flame will assume "default" which is not defined in the model. All agents should start in this state. 
        people = FLAMEGPU.agent("person", "waiting");
        
        # Get population of agents which may have been loaded from an external configuration file
        if(people.count() > 0):
            # People have been pre-defined with input file. Set up additional variables for them here, but assume their position is already set
            # Get DeviceAgentVector to the pedestrian population
            txt = f"Creating people agents from config file\n"
            print(txt)

            t_vector = people.getPopulationData()
            
            for t in t_vector:    
                
                t.setVariableFloat("m", m_ped)
                t.setVariableFloat("n", n_ped)
                t.setVariableFloat("p", p_ped)
                t.setVariableFloat("cap", cap_ped)
                t.setVariableFloat("steering", steering_ped * (FLAMEGPU.random.uniformInt(8, 12)/10))
                t.setVariableFloat("minSep", 1000)
                
        else:
            txt = f"Creating people agents for modelling run\n"
            print(txt)

            # Agent choose a wall to lean on
            def choose_wall_waiting_location():
                r =  FLAMEGPU.random.uniformInt(0, 99)

                if r < 5:
                    # Passengers prefer to stay away from the main pedestrian flow while waiting, 5%
                    x = FLAMEGPU.random.uniformInt(50, 350) / 10.0 
                    y = FLAMEGPU.random.uniformInt(3, 5) / 10.0

                elif r < 10:
                    # upper wall: platform edge, Passengers tend to maintain distance from it, 5%, at least 1.5m from the edge
                    x = FLAMEGPU.random.uniformInt(50, 350) / 10.0
                    y = FLAMEGPU.random.uniformInt(60, 65) / 10.0

                elif r < 55:
                    # left wall, 45%
                    x = FLAMEGPU.random.uniformInt(3, 8) / 10.0
                    y = FLAMEGPU.random.uniformInt(35, 70) / 10.0

                else:
                    # right wall, 45%
                    x = FLAMEGPU.random.uniformInt(392, 397) / 10.0
                    y = FLAMEGPU.random.uniformInt(35, 70) / 10.0

                return x, y


            #Start people that waits
            for i in range(20):
                t = people.newAgent()

                #devide waiting people into two groups: left and right
                if i < 5:
                    start_x = 0.8
                    velx = 1.0
                else:
                    start_x = 39.2
                    velx = -1.0

                start_y = FLAMEGPU.random.uniformInt(0, 30) / 10.0
                wait_x, wait_y = choose_wall_waiting_location() #choose one wall and its coordinate

                t.setVariableFloat("x", start_x)
                t.setVariableFloat("y", start_y)

                # final destination
                t.setVariableFloat("destx", 0, wait_x)
                t.setVariableFloat("desty", 0, wait_y)
                t.setVariableInt("dwell", 0, 0)

                # waiting point as waypoint 
                t.setVariableFloat("destx", 1, wait_x)
                t.setVariableFloat("desty", 1, wait_y)
                t.setVariableInt("dwell", 1, 99999) #will wait here thorugh the whole simulation
                t.setVariableInt("journeyIndex", 1)    
                t.setVariableFloat("velx", velx)
                t.setVariableFloat("vely", 0.0)
                t.setVariableFloat("steering", steering_ped * (FLAMEGPU.random.uniformInt(8, 12)/10))
                t.setVariableInt("starttime", FLAMEGPU.random.uniformInt(0, 5000))
                t.setVariableFloat("maxSpeed", FLAMEGPU.random.uniformInt(12,17)/10.0)
                t.setVariableFloat("m", m_ped)
                t.setVariableFloat("n", n_ped)
                t.setVariableFloat("p", p_ped)
                t.setVariableFloat("cap", cap_ped)
                t.setVariableInt("dwelltime", 0)
                t.setVariableFloat("minSep", 1000.0)

            #Start people that walks
            for j in range(25):
                t = people.newAgent()

                if j < 10:
                # left door -> right door
                    start_x = 0.8
                    dest_x= 43.0 #destination
                    velx = 1.0
                else:
                # right door -> left door
                    start_x = 39.2
                    dest_x= -3.0
                    velx = -1.0

                start_y = FLAMEGPU.random.uniformInt(0, 30) / 10.0
                dest_y = FLAMEGPU.random.uniformInt(0, 30) / 10.0

                t.setVariableFloat("x", start_x)
                t.setVariableFloat("y", start_y)
                t.setVariableFloat("destx", 0, dest_x)
                t.setVariableFloat("desty", 0, dest_y)
                t.setVariableInt("dwell", 0, 0)
                t.setVariableInt("journeyIndex", 0)
                t.setVariableFloat("velx", velx)
                t.setVariableFloat("vely", 0.0)  
                t.setVariableFloat("steering", steering_ped * (FLAMEGPU.random.uniformInt(8, 12)/10))
                t.setVariableInt("starttime", FLAMEGPU.random.uniformInt(0, 5000))
                t.setVariableFloat("maxSpeed", FLAMEGPU.random.uniformInt(12,17)/10.0)
                t.setVariableFloat("m", m_ped)
                t.setVariableFloat("n", n_ped)
                t.setVariableFloat("p", p_ped)
                t.setVariableFloat("cap", cap_ped)
                t.setVariableInt("dwelltime", 0)
                t.setVariableFloat("minSep", 1000.0)
                i+=1

            txt = f"Created {i} people agents\n"
            print(txt)


class InitObservers(pyflamegpu.HostFunction):
    def run(self, FLAMEGPU):
        i = 0;
        count = 0;
        
        txt = f"Creating observer agents\n"
        print(txt)
        
        observer = FLAMEGPU.agent("observer")

        for xx in range(50):
    
            for yy in range(50):

                t = observer.newAgent()
                t.setVariableFloat("x", xx/0.5)
                t.setVariableFloat("y", yy/0.5)
                t.setVariableFloat("cap", 1.0)
                t.setVariableFloat("totalfield", 0.0)
                i+=1


        txt = f"Created {i} observer agents\n"
        print(txt)



# Define the FLAME GPU model
model = pyflamegpu.ModelDescription("Pedestrian movement example")

# //////Messaging and communications //
location_message = model.newMessageSpatial2D("location_message")
location_message.setMin(0, 0)
location_message.setMax(50.0, 50.0)
location_message.setRadius(1.0)
location_message.newVariableFloat("m");
location_message.newVariableFloat("n");
location_message.newVariableFloat("p");
location_message.newVariableInt("type");
location_message.newVariableID("id");

# //////Define agents ////////////////
wall = model.newAgent("wall")
wall.newVariableFloat("x")
wall.newVariableFloat("y")
wall.newVariableFloat("m")
wall.newVariableFloat("n")
wall.newVariableFloat("p")

person = model.newAgent("person")
person.newVariableFloat("x")
person.newVariableFloat("y")
person.newVariableArrayFloat("destx", 10)
person.newVariableArrayFloat("desty", 10)
person.newVariableArrayInt("dwell", 10)
person.newVariableInt("dwelltime")
person.newVariableInt("journeyIndex")
person.newVariableFloat("velx")
person.newVariableFloat("vely")
person.newVariableFloat("steering")
person.newVariableInt("starttime")
person.newVariableFloat("maxSpeed")
person.newVariableFloat("m")
person.newVariableFloat("n")
person.newVariableFloat("p")
person.newVariableFloat("cap")
person.newVariableFloat("minSep")
person.newState("waiting")
person.newState("moving")
person.newState("arrived")

person.setInitialState("waiting")

# An observer agent - use for exploring fields and debugging, but does not influence behaviour of other agents
observer = model.newAgent("observer")
observer.newVariableFloat("x")
observer.newVariableFloat("y")
observer.newVariableFloat("cap")
observer.newVariableFloat("totalfield")

# Attaching message types to agent functions (more than one agent type can contribute to a message queue)

# Emit messages from all obstacles / walls
wall_message_fn = wall.newRTCFunction("emit_messages_walls", emit_messages_walls)
wall_message_fn.setMessageOutput("location_message")

# Start movement of people at their starttime. 
start_fn = person.newRTCFunction("activate_people", start)
start_fn.setInitialState("waiting")
start_fn.setEndState("moving")
start_fn.setRTCFunctionCondition(starttime)

# Emit messages from moving(active) people, but not those prior to movement or after arrival at their destination. 
person_message_fn = person.newRTCFunction("emit_messages_people", emit_messages_people)
person_message_fn.setInitialState("moving");
person_message_fn.setMessageOutput("location_message")
person_message_fn.setEndState("moving")

# Once messages from fixed objects (walls) and people are available, decisions can be made on moving people
people_move_fn = person.newRTCFunction("move_people", move_people)
people_move_fn.setInitialState("moving")
people_move_fn.setMessageInput("location_message")
people_move_fn.setEndState("moving")

# Explore the field controlling movement - record it's state for later plotting
observer_fn = observer.newRTCFunction("check_field", check_field)
observer_fn.setMessageInput("location_message")

# Once an agent arrives at their destination, remove them from the simulation 
arrived_fn = person.newRTCFunction("arrived", arrived)
arrived_fn.setInitialState("moving")
arrived_fn.setEndState("arrived")
arrived_fn.setRTCFunctionCondition(destination)
  

# // global environment variables
env = model.Environment();

# All units in the simulation must be self-consistent 
env.newPropertyInt("visualisation", 0)
env.newPropertyArrayInt("camera", [0, 0, 0, 0, 0, 0])
env.newPropertyInt("logging", 0)
env.newPropertyInt("debug", 0)
env.newPropertyFloat("timestep", 0.5)
env.newPropertyFloat("weighting", 1)
env.newPropertyInt("checkfield_cycles", 0)
env.newPropertyInt("checkfield_type", 0)

env.newPropertyFloat("m_ped", 2.0)
env.newPropertyFloat("n_ped", 10.0)
env.newPropertyFloat("p_ped", 2.0)
env.newPropertyFloat("cap_ped", 1.0)
env.newPropertyFloat("steering_ped", 0.1745)
env.newPropertyFloat("lookahead_ped", 0.5)
env.newPropertyFloat("accuracy_ped", 0.75)
env.newPropertyFloat("waypoint_accuracy_ped", 0.25)

env.newPropertyFloat("m_wall", 2.0)
env.newPropertyFloat("n_wall", 10.0)
env.newPropertyFloat("p_wall", 2.0)


# Setup execution order
model.addInitFunction(InitWalls())
model.addInitFunction(InitPeople())
model.addInitFunction(InitObservers())


# Identify the root of execution
model.addExecutionRoot(wall_message_fn)
start_fn.dependsOn(wall_message_fn)
person_message_fn.dependsOn(start_fn)
# Record field observation prior to people moving, but using the same information that will control them
observer_fn.dependsOn(person_message_fn)
people_move_fn.dependsOn(person_message_fn)
arrived_fn.dependsOn(people_move_fn) 

# Build the model
model.generateLayers()
  
# Visualise the model confugiration
model.generateDependencyGraphDOTDiagram("graphdiagram.gv")
  
# Convert model to a simulation
simulation = pyflamegpu.CUDASimulation(model)
simulation.initialise(sys.argv)

# Specify the desired Exit LoggingConfig
exit_log_cfg = pyflamegpu.LoggingConfig(model)
exit_log_cfg.logEnvironment("camera")
exit_log_cfg.logEnvironment("visualisation")
exit_log_cfg.logEnvironment("logging")
simulation.setExitLog(exit_log_cfg)

# Avoid messages about items not defined in the XML config file
# simulation.SimulationConfig().verbosity = flamegpu::Verbosity::Quiet;
  

# Only run this block if pyflamegpu was built with visualisation support
if pyflamegpu.VISUALISATION:
    # Create visualisation
    
    if(simulation.getEnvironmentPropertyInt("visualisation") >= 1):
        print("Setting up visualisation\n")
        
        visualiser = simulation.getVisualisation()

        visualiser.setInitialCameraTarget(simulation.getEnvironmentPropertyInt("camera",0),
				          simulation.getEnvironmentPropertyInt("camera",1), simulation.getEnvironmentPropertyInt("camera",2))
        visualiser.setInitialCameraLocation(simulation.getEnvironmentPropertyInt("camera",3),
					    simulation.getEnvironmentPropertyInt("camera",4), simulation.getEnvironmentPropertyInt("camera",5))
        visualiser.setCameraSpeed(0.01)
        
        # Add "wall" agents to the visualisation
        wall_agt = visualiser.addAgent("wall")
        # Location variables have names "x" and "y" so will be used by default
        wall_agt.setModel(pyflamegpu.CUBE)
        wall_agt.setColor(pyflamegpu.RED)
        wall_agt.setModelScale(0.1, 0.1, 0.1)
        wall_agt.setXVariable("x")
        wall_agt.setYVariable("y")
        
        # Add "people" agents to the visualisation
        person_agt = visualiser.addAgent("person")
        # Location variables have names "x" and "y" so will be used by default
        person_agt.setModel(pyflamegpu.SPHERE)
        person_agt.State("waiting").setColor(pyflamegpu.WHITE)
        person_agt.State("arrived").setColor(pyflamegpu.GREEN)
        person_agt.State("moving").setColor(pyflamegpu.BLUE)
        person_agt.setModelScale(0.25, 0.25, 0.25)
        person_agt.setXVariable("x")
        person_agt.setYVariable("y")
        
        # Remove from the visualisation agents that have not started, or have arrived at their destination
        waiting_person = person_agt.State("waiting")
        moving_person = person_agt.State("moving")
        arrived_person = person_agt.State("arrived")
        person_agt.State("waiting").setVisible(True)
        person_agt.State("moving").setVisible(True)
        person_agt.State("arrived").setVisible(True)
        waiting_person.setVisible(True)
        moving_person.setVisible(True)
        arrived_person.setVisible(True)
        
        # Open the visualiser window
        visualiser.activate();
        
# Run the simulation
simulation.simulate()


#Exit logging
# -*- coding: utf-8 -*-

from datetime import datetime
import os
import pyflamegpu
import sys

simulation = pyflamegpu.CUDASimulation(model)
simulation.initialise(sys.argv)

# Run init host functions explicitly once
simulation.initFunctions()

visualiser = None
if pyflamegpu.VISUALISATION == 1:
    visualiser = simulation.getVisualisation()
    visualiser.activate()

total_steps = simulation.SimulationConfig().steps
save_every = 100

run_stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
output_dir = "time_states_" + run_stamp
os.makedirs(output_dir, exist_ok=True)

