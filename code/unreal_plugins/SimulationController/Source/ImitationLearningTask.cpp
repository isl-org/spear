#include "ImitationLearningTask.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <EngineUtils.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>
#include <UObject/UObjectGlobals.h>

#include "ActorHitEvent.h"
#include "Assert/Assert.h"
#include "Box.h"
#include "Config.h"

ImitationLearningTask::ImitationLearningTask(UWorld* world)
{
    FActorSpawnParameters goal_spawn_params;
    goal_spawn_params.Name = FName(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}).c_str());
    goal_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    goal_actor_ = world->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, goal_spawn_params);
    ASSERT(goal_actor_);

    auto scene_component = NewObject<USceneComponent>(goal_actor_);
    scene_component->SetMobility(EComponentMobility::Movable);
    goal_actor_->SetRootComponent(scene_component);

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // Create UActorHitEvent but don't subscribe to any actors yet
    actor_hit_event_ = NewObject<UActorHitEvent>(new_object_parent_actor_, TEXT("ActorHitEvent"));
    ASSERT(actor_hit_event_);
    actor_hit_event_->RegisterComponent();
    actor_hit_event_delegate_handle_ = actor_hit_event_->delegate_.AddRaw(this, &ImitationLearningTask::actorHitEventHandler);

    // If the start/goal positions are not randomly generated, get them from a file
    if (!Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GET_POSITIONS_FROM_TRAJECTORY_SAMPLING"})) {
        getPositionsFromFile();
    }

    position_index_ = 0;
    hit_goal_ = false;
    hit_obstacle_ = false;
}

ImitationLearningTask::~ImitationLearningTask()
{
    hit_obstacle_ = false;
    hit_goal_ = false;
    position_index_ = -1;

    cleanUpPositions();
    
    ASSERT(actor_hit_event_);
    actor_hit_event_->delegate_.Remove(actor_hit_event_delegate_handle_);
    actor_hit_event_delegate_handle_.Reset();
    actor_hit_event_->DestroyComponent();
    actor_hit_event_ = nullptr;

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;

    ASSERT(goal_actor_);
    goal_actor_->Destroy();
    goal_actor_ = nullptr;
}

void ImitationLearningTask::findObjectReferences(UWorld* world)
{
    auto obstacle_ignore_actor_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "OBSTACLE_IGNORE_ACTOR_NAMES"});
    for (TActorIterator<AActor> actor_itr(world); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*((*actor_itr)->GetName()));
        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "AGENT_ACTOR_NAME"})) {
            ASSERT(!agent_actor_);
            agent_actor_ = *actor_itr;
        } else if (std::find(obstacle_ignore_actor_names.begin(), obstacle_ignore_actor_names.end(), actor_name) != obstacle_ignore_actor_names.end()) {
            obstacle_ignore_actors_.emplace_back(*actor_itr);
            std::cout << "Adding actor to obstacle ignore list: " << actor_name << std::endl;
        }
    }
    ASSERT(agent_actor_);

    // Subscribe to the agent actor now that we have obtained a reference to it
    actor_hit_event_->subscribeToActor(agent_actor_);

    nav_sys_ = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
    ASSERT(nav_sys_);

    auto nav_agent_interface = dynamic_cast<INavAgentInterface*>(agent_actor_);
    ASSERT(nav_agent_interface);

    nav_data_ = nav_sys_->GetNavDataForProps(nav_agent_interface->GetNavAgentPropertiesRef(), nav_agent_interface->GetNavAgentLocation());
    ASSERT(nav_data_);
}

void ImitationLearningTask::cleanUpObjectReferences()
{
    ASSERT(nav_data_);
    nav_data_ = nullptr;

    ASSERT(nav_sys_);
    nav_sys_ = nullptr;

    ASSERT(actor_hit_event_);
    actor_hit_event_->unsubscribeFromActor(agent_actor_);

    obstacle_ignore_actors_.clear();

    ASSERT(agent_actor_);
    agent_actor_ = nullptr;
}

void ImitationLearningTask::beginFrame()
{
    hit_goal_ = false;
    hit_obstacle_ = false;
}

void ImitationLearningTask::endFrame()
{
}

float ImitationLearningTask::getReward() const
{
    return -std::numeric_limits<float>::infinity();
}

bool ImitationLearningTask::isEpisodeDone() const
{
    return hit_goal_ || hit_obstacle_;
}

std::map<std::string, Box> ImitationLearningTask::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
    step_info_space["hit_goal"] = std::move(box);

    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
    step_info_space["hit_obstacle"] = std::move(box);

    return step_info_space;
}

std::map<std::string, std::vector<uint8_t>> ImitationLearningTask::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    step_info["hit_goal"] = std::vector<uint8_t>{hit_goal_};
    step_info["hit_obstacle"] = std::vector<uint8_t>{hit_obstacle_};

    return step_info;
}

void ImitationLearningTask::reset()
{
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GET_POSITIONS_FROM_TRAJECTORY_SAMPLING"})) {
        getPositionsFromTrajectorySampling();
    }

    // Set agent and goal positions
    bool sweep = false;
    FHitResult* hit_result_info = nullptr;
    agent_actor_->SetActorLocation(agent_initial_positions_.at(position_index_), sweep, hit_result_info, ETeleportType::ResetPhysics);
    goal_actor_->SetActorLocation(agent_goal_positions_.at(position_index_), sweep, hit_result_info, ETeleportType::ResetPhysics);
    
    // Increment position_index_
    if (position_index_ < agent_goal_positions_.size() - 1) { 
        position_index_++;
    }  else {
        position_index_ = 0;
    }
}

bool ImitationLearningTask::isReady() const
{
    return true;
}

void ImitationLearningTask::actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
{
    ASSERT(self_actor == agent_actor_);

    if (other_actor == goal_actor_) {
        hit_goal_ = true;
    } else if (std::find(obstacle_ignore_actors_.begin(), obstacle_ignore_actors_.end(), other_actor) == obstacle_ignore_actors_.end()) {
        hit_obstacle_ = true;
    }
}

void ImitationLearningTask::getPositionsFromFile()
{
    std::string line;
    std::string token;
    FVector init, goal;

    // Create an input filestream 
    std::ifstream fs(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "POSITIONS_FILE"})); 
    ASSERT(fs.is_open());

    // Read file data, line by line in the format "init.X, init.Y, init.Z, goal.X, goal.Y, goal.Z"
    std::getline(fs, line); // get header
    while (std::getline(fs, line)) {
        std::istringstream ss(line);
        std::getline(ss, token, ','); ASSERT(ss); init.X = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); init.Y = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); init.Z = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); goal.X = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); goal.Y = std::stof(token);
        std::getline(ss, token, ','); ASSERT(ss); goal.Z = std::stof(token);
        agent_initial_positions_.push_back(init);
        agent_goal_positions_.push_back(goal);
    }

    // Close file
    fs.close();
}

void ImitationLearningTask::getPositionsFromTrajectorySampling()
{
    // Sanity checks
    ASSERT(nav_data_);
    ASSERT(nav_sys_);

    agent_initial_positions_.clear();
    agent_goal_positions_.clear();

    float best_path_score = 0.0f;
    FNavLocation best_init_location;
    FNavLocation best_target_location;
    TArray<FNavPathPoint> best_path_points;

    // Path generation polling to get "interesting" paths in every experiment
    for (int i = 0; i < Config::getValue<int>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "TRAJECTORY_SAMPLING_MAX_ITERS"}); i++) {

        FNavLocation init_location;
        FNavLocation target_location;
        bool found = false;

        // Get a random initial point
        found = nav_sys_->GetRandomPoint(init_location, nav_data_);
        ASSERT(found);

        // Get a random reachable target point, to be reached by the agent from init_location.Location
        found = nav_sys_->GetRandomReachablePointInRadius(init_location.Location, Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "TRAJECTORY_SAMPLING_SEARCH_RADIUS"}), target_location);
        ASSERT(found);

        // Update navigation query with the new target
        FPathFindingQuery nav_query = FPathFindingQuery(agent_actor_, *nav_data_, init_location.Location, target_location.Location);

        // Generate a collision-free path between the initial position and the target position
        FPathFindingResult collision_free_path = nav_sys_->FindPathSync(nav_query, EPathFindingMode::Type::Regular);

        // If path generation is sucessful, make sure that it is not too simple for better training results
        if (collision_free_path.IsSuccessful() && collision_free_path.Path.IsValid()) {

            // Debug output
            if (collision_free_path.IsPartial()) {
                std::cout << "Only a partial path could be found..." << std::endl; // Partial paths are supported; in this case, the agent is expected to move as close as possible to its target
            }

            // Compute a path score to evaluate its complexity
            int num_waypoints = collision_free_path.Path->GetPathPoints().Num();
            FVector2D relative_position_to_target((target_location.Location - init_location.Location).X, (target_location.Location - init_location.Location).Y);
            float path_score = relative_position_to_target.Size() * num_waypoints;

            // If the path_score is the best we've seen so far, update best_init_location and best_target_location
            if (best_path_score <= path_score) {
                best_path_score = path_score;
                best_init_location = init_location;
                best_target_location = target_location;
                best_path_points = collision_free_path.Path->GetPathPoints();

                // Debug output
                float trajectory_length = 0.0;
                for (int j = 0; j < num_waypoints - 1; j++) {
                    trajectory_length += FVector::Dist(best_path_points[j].Location, best_path_points[j + 1].Location);
                }
                std::cout << "Iteration: " << i << std::endl;
                std::cout << "Score: " << best_path_score << std::endl;
                std::cout << "Number of waypoints: " << num_waypoints << std::endl;
                std::cout << "Target distance: " << relative_position_to_target.Size() / agent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;
                std::cout << "Path length: " << trajectory_length / agent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters << "m" << std::endl;
            }

            // Debug output
            if (i % 1000 == 0) {
                std::cout << ".";
            }
        }
    }

    ASSERT(best_path_points.Num() > 1);

    // Update positions
    agent_initial_positions_.push_back(best_init_location.Location);
    agent_goal_positions_.push_back(best_target_location.Location);

    // Debug output
    std::cout << std::endl;
    std::cout << "Initial position: [" << agent_initial_positions_.at(0).X << ", " << agent_initial_positions_.at(0).Y << ", " << agent_initial_positions_.at(0).Z << "]." << std::endl;
    std::cout << "Reachable position: [" << agent_goal_positions_.at(0).X << ", " << agent_goal_positions_.at(0).Y << ", " << agent_goal_positions_.at(0).Z << "]." << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "Waypoints: " << std::endl;
    for (auto& point : best_path_points) {
        std::cout << "[" << point.Location.X << ", " << point.Location.Y << ", " << point.Location.Z << "]" << std::endl;
    }
    std::cout << "-----------------------------------------------------------" << std::endl;
}

void ImitationLearningTask::cleanUpPositions()
{
    agent_initial_positions_.clear();
    agent_goal_positions_.clear();
}
