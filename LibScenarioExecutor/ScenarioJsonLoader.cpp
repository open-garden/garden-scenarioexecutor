#include <string>
#include <sstream>
#include <fstream>
#include "./picojson/picojson.h"
#include "ScenarioJsonLoader.hpp"

void resetActionExec(TAction *action) {
    action->traveled_distance.exec = false;
    action->speed.exec = false;
    action->lane_change.exec = false;
    action->lane_offset.exec = false;
    action->route_move.exec = false;
    action->light_state.exec = false;
    action->light_state.blinker_left = -1;
    action->light_state.blinker_right = -1;
    action->light_state.special1 = -1;
    action->sound_state.exec = false;
    action->sound_state.sound = -1;
    action->hand_over.exec = false;
}

void resetConditionExec(TExp *exp) {
    exp->position.exec = false;
    exp->distance.exec = false;
    exp->speed.exec = false;
    exp->time_headway.exec = false;
    exp->simulation_time.exec = false;
}

void setActor(TActor *actor, picojson::object actor_) {
    actor->actor_id = (int)actor_["actor_id"].get<double>();
    actor->model_id = actor_["model_id"].get<string>();
    if (!actor_["color"].is<picojson::null>()) {
        actor->color = actor_["color"].get<string>();
    } else {
        actor->color = "000000";
    }
}

void setActors(TActors *actors, picojson::object actors_) {
    setActor(&actors->ego, actors_["ego"].get<picojson::object>());

    if (!actors_["others"].is<picojson::null>()) {
        for (picojson::value other_: actors_["others"].get<picojson::array>()) {
            TActor other;
            setActor(&other, other_.get<picojson::object>());
            actors->others.push_back(other);
        }
    }
}

void setNextScenes(vector<int> *next_scenes, picojson::array next_scenes_) {
    for (picojson::value scene: next_scenes_) {
        (*next_scenes).push_back((int)scene.get<double>());
    }
}

void setPosition(TPosition *position, picojson::object position_) {
    position->exec = true;
    position->type = position_["type"].get<string>();
    position->actor_id = (int)position_["actor_id"].get<double>();
    position->wp_id = position_["wp_id"].get<string>();
    position->wp_idx = (int)position_["wp_idx"].get<double>();
    position->tolerance = position_["tolerance"].get<double>();
}

void setSpeed(TSpeed *speed, picojson::object speed_) {
    speed->exec = true;
    if (!speed_["actor_id"].is<picojson::null>()) {
        speed->actor_id = (int)speed_["actor_id"].get<double>();
    }
    speed->type = speed_["type"].get<string>();
    if (!speed_["target_actor_id"].is<picojson::null>()) {
        speed->target_actor_id = (int)speed_["target_actor_id"].get<double>();
    }
    if (!speed_["comparison"].is<picojson::null>()) {
        speed->comparison = speed_["comparison"].get<string>();
    }
    speed->value = speed_["value"].get<double>();
    if (!speed_["accel"].is<picojson::null>()) {
        picojson::object accel = speed_["accel"].get<picojson::object>();
        speed->accel.type = accel["type"].get<string>();
        speed->accel.value = accel["value"].get<double>();
    } else {
        speed->accel.type = "god_hands";
    }
}

void setDistance(TDistance *distance, picojson::object distance_) {
    distance->exec = true;
    distance->actor_id = (int)distance_["actor_id"].get<double>();
    if (!distance_["type"].is<picojson::null>()) {
        distance->type = distance_["type"].get<string>();
    } else {
        distance->type = "traveled";
    }
    distance->target_actor_id = (int)distance_["target_actor_id"].get<double>();
    if (!distance_["comparison"].is<picojson::null>()) {
        distance->comparison = distance_["comparison"].get<string>();
    }
    distance->value = distance_["value"].get<double>();
    if (!distance_["measure_type"].is<picojson::null>()) {
        distance->measure_type = distance_["measure_type"].get<string>();
    } else {
        distance->measure_type = "center";
    }
}

void setTraveledDistance(TTraveledDistance *traveled_distance, picojson::object traveled_distance_) {
    traveled_distance->exec = true;
    traveled_distance->actor_id = (int)traveled_distance_["actor_id"].get<double>();
    traveled_distance->value = traveled_distance_["value"].get<double>();
    if (!traveled_distance_["measure_type"].is<picojson::null>()) {
        traveled_distance->measure_type = traveled_distance_["measure_type"].get<string>();
    } else {
        traveled_distance->measure_type = "center";
    }
}

void setLaneChange(TLaneChange *lane_change, picojson::object lane_change_) {
    lane_change->exec = true;
    lane_change->actor_id = (int)lane_change_["actor_id"].get<double>();
    lane_change->type = lane_change_["type"].get<string>();
    lane_change->wp_id = lane_change_["wp_id"].get<string>();
    lane_change->time = lane_change_["time"].get<double>();
}

void setLaneOffset(TLaneOffset *lane_offset, picojson::object lane_offset_) {
    lane_offset->exec = true;
    lane_offset->actor_id = (int)lane_offset_["actor_id"].get<double>();
    lane_offset->value = lane_offset_["value"].get<double>();
    lane_offset->time = lane_offset_["time"].get<double>();
}

void setRouteMove(TRouteMove *route_move, picojson::object route_move_) {
    route_move->exec = true;
    route_move->actor_id = (int)route_move_["actor_id"].get<double>();
    route_move->type = route_move_["type"].get<string>();
    picojson::array routes = route_move_["route"].get<picojson::array>();
    for (picojson::value route_: routes) {
        TRoute route;
        route.time = route_.get<picojson::object>()["time"].get<double>();
        if (!route_.get<picojson::object>()["wp_id"].is<picojson::null>()) {
            route.wp_id = route_.get<picojson::object>()["wp_id"].get<string>();
        }
        if (!route_.get<picojson::object>()["wp_idx"].is<picojson::null>()) {
            route.wp_idx = (int)route_.get<picojson::object>()["wp_idx"].get<double>();
        }
        if (!route_.get<picojson::object>()["wp_offset"].is<picojson::null>()) {
            route.wp_offset = route_.get<picojson::object>()["wp_offset"].get<double>();
        }
        route_move->route.push_back(route);
    }
}

void setLightState(TLightState *light_state, picojson::object light_state_) {
    light_state->exec = true;
    light_state->actor_id = (int)light_state_["actor_id"].get<double>();
    if (!light_state_["blinker_left"].is<picojson::null>()) {
        light_state->blinker_left = (int)light_state_["blinker_left"].get<bool>();
    }
    if (!light_state_["blinker_right"].is<picojson::null>()) {
        light_state->blinker_right = (int)light_state_["blinker_right"].get<bool>();
    }
    if (!light_state_["special1"].is<picojson::null>()) {
        light_state->special1 = (int)light_state_["special1"].get<bool>();
    }
}

void setSoundState(TSoundState *sound_state, picojson::object sound_state_) {
    sound_state->exec = true;
    sound_state->actor_id = (int)sound_state_["actor_id"].get<double>();
    if (!sound_state_["sound"].is<picojson::null>()) {
        sound_state->sound = (int)sound_state_["sound"].get<bool>();
    }
}

void setHandOver(THandOver *hand_over, picojson::object hand_over_) {
    hand_over->exec = true;
    hand_over->hand_over = hand_over_["hand_over"].get<bool>();
    if (!hand_over_["throttle"].is<picojson::null>()) {
        hand_over->throttle = hand_over_["throttle"].get<double>();
    }
    if (!hand_over_["brake"].is<picojson::null>()) {
        hand_over->brake = hand_over_["brake"].get<double>();
    }
    if (!hand_over_["steer"].is<picojson::null>()) {
        hand_over->steer = hand_over_["steer"].get<double>();
    }
}

void setOpeningScene(TOpeningScene *opening_scene, picojson::object opening_scene_) {
    picojson::object ego = opening_scene_["ego"].get<picojson::object>();
    picojson::object start_position = ego["start_position"].get<picojson::object>();
    opening_scene->ego.start_position.wp_id = start_position["wp_id"].get<string>();
    opening_scene->ego.start_position.wp_idx = (int)start_position["wp_idx"].get<double>();

    if (!ego["end_position"].is<picojson::null>()) {
        picojson::object end_position = ego["end_position"].get<picojson::object>();
        opening_scene->ego.end_position.wp_id = end_position["wp_id"].get<string>();
        opening_scene->ego.end_position.wp_idx = (int)end_position["wp_idx"].get<double>();
    }

    if (!opening_scene_["others"].is<picojson::null>()) {
        picojson::array others = opening_scene_["others"].get<picojson::array>();
        for (picojson::value other_: others) {
            TOpeningAction other;
            other.actor_id = (int)other_.get<picojson::object>()["actor_id"].get<double>();
            picojson::object start_position = other_.get<picojson::object>()["start_position"].get<picojson::object>();
            other.start_position.type = start_position["type"].get<string>();
            other.start_position.wp_id = start_position["wp_id"].get<string>();
            if (!start_position["wp_idx"].is<picojson::null>()) {
                other.start_position.wp_idx = (int)start_position["wp_idx"].get<double>();
            }
            if (!start_position["wp_offset"].is<picojson::null>()) {
                other.start_position.wp_offset = start_position["wp_offset"].get<double>();
            } else {
                other.start_position.wp_offset = 0.0;
            }
            if (!start_position["orientation"].is<picojson::null>()) {
                picojson::object orientation = start_position["orientation"].get<picojson::object>();
                other.start_position.orientation.heading = orientation["heading"].get<double>();
            }
            if (!start_position["distance"].is<picojson::null>()) {
                other.start_position.distance = start_position["distance"].get<double>();
            }
            if (!start_position["measure_type"].is<picojson::null>()) {
                other.start_position.measure_type = start_position["measure_type"].get<string>();
            }
            resetActionExec(&other.action);
            if (!other_.get<picojson::object>()["start_speed"].is<picojson::null>()) {
                setSpeed(&other.action.speed, other_.get<picojson::object>()["start_speed"].get<picojson::object>());
                other.action.speed.actor_id = other.actor_id;
            }
            opening_scene->others.push_back(other);
        }
    }

    setNextScenes(&opening_scene->next_scenes, opening_scene_["next_scenes"].get<picojson::array>());
}

void setTimeHeadway(TTimeHeadway *time_headway, picojson::object time_headway_) {
    time_headway->exec = true;
    time_headway->actor_id = (int)time_headway_["actor_id"].get<double>();
    time_headway->wp_id = time_headway_["wp_id"].get<string>();
    time_headway->wp_idx = (int)time_headway_["wp_idx"].get<double>();
    time_headway->comparison = time_headway_["comparison"].get<string>();
    time_headway->value = time_headway_["value"].get<double>();
}

void setSimulationTime(TSimulationTime *simulation_time, picojson::object simulation_time_) {
    simulation_time->exec = true;
    simulation_time->comparison = simulation_time_["comparison"].get<string>();
    simulation_time->value = simulation_time_["value"].get<double>();
}

void setCollision(TCollision *collision, picojson::object collision_) {
    collision->exec = true;
}

void setExp(vector<TExp> *exps, picojson::array exps_) {
    for (picojson::value exp_: exps_) {
        TExp exp;
        resetConditionExec(&exp);
        if (!exp_.get<picojson::object>()["position"].is<picojson::null>()) {
            setPosition(&exp.position, exp_.get<picojson::object>()["position"].get<picojson::object>());
        }
        else if (!exp_.get<picojson::object>()["speed"].is<picojson::null>()) {
            setSpeed(&exp.speed, exp_.get<picojson::object>()["speed"].get<picojson::object>());
        }
        else if (!exp_.get<picojson::object>()["distance"].is<picojson::null>()) {
            setDistance(&exp.distance, exp_.get<picojson::object>()["distance"].get<picojson::object>());
        }
        else if (!exp_.get<picojson::object>()["time_headway"].is<picojson::null>()) {
            setTimeHeadway(&exp.time_headway, exp_.get<picojson::object>()["time_headway"].get<picojson::object>());
        }
        else if (!exp_.get<picojson::object>()["simulation_time"].is<picojson::null>()) {
            setSimulationTime(&exp.simulation_time, exp_.get<picojson::object>()["simulation_time"].get<picojson::object>());
        }
        else if (!exp_.get<picojson::object>()["collision"].is<picojson::null>()) {
            setCollision(&exp.collision, exp_.get<picojson::object>()["collision"].get<picojson::object>());
        }
        (*exps).push_back(exp);
    }
}    

void setConditions(TConditions *conditions, picojson::object conditions_) {
    if (!conditions_["and"].is<picojson::null>()) {
        setExp(&conditions->and_, conditions_["and"].get<picojson::array>());
    }
    if (!conditions_["or"].is<picojson::null>()) {
        setExp(&conditions->or_, conditions_["or"].get<picojson::array>());
    }
}

void setActions(vector<TAction> *actions, picojson::array actions_) {
    for (picojson::value action_: actions_) {
        TAction action;
        resetActionExec(&action);
        if (!action_.get<picojson::object>()["traveled_distance"].is<picojson::null>()) {
            setTraveledDistance(&action.traveled_distance, action_.get<picojson::object>()["traveled_distance"].get<picojson::object>());
        } else if (!action_.get<picojson::object>()["speed"].is<picojson::null>()) {
            setSpeed(&action.speed, action_.get<picojson::object>()["speed"].get<picojson::object>());
        } else if (!action_.get<picojson::object>()["lane_change"].is<picojson::null>()) {
            setLaneChange(&action.lane_change, action_.get<picojson::object>()["lane_change"].get<picojson::object>());
        } else if (!action_.get<picojson::object>()["lane_offset"].is<picojson::null>()) {
            setLaneOffset(&action.lane_offset, action_.get<picojson::object>()["lane_offset"].get<picojson::object>());
        } else if (!action_.get<picojson::object>()["route_move"].is<picojson::null>()) {
            setRouteMove(&action.route_move, action_.get<picojson::object>()["route_move"].get<picojson::object>());
        } else if (!action_.get<picojson::object>()["light_state"].is<picojson::null>()) {
            setLightState(&action.light_state, action_.get<picojson::object>()["light_state"].get<picojson::object>());
        } else if (!action_.get<picojson::object>()["sound_state"].is<picojson::null>()) {
            setSoundState(&action.sound_state, action_.get<picojson::object>()["sound_state"].get<picojson::object>());
        } else if (!action_.get<picojson::object>()["hand_over"].is<picojson::null>()) {
            setHandOver(&action.hand_over, action_.get<picojson::object>()["hand_over"].get<picojson::object>());
        }
        (*actions).push_back(action);
    }
}

void setScenes(vector<TScene> *scenes, picojson::array scenes_) {
    for (picojson::value scene_: scenes_) {
        TScene scene;
        scene.scene_id = (int)scene_.get<picojson::object>()["scene_id"].get<double>();
        if (!scene_.get<picojson::object>()["duration"].is<picojson::null>()) {
            scene.duration = scene_.get<picojson::object>()["duration"].get<double>();
        } else {
            scene.duration = 0.0;
        }
        if (!scene_.get<picojson::object>()["conditions"].is<picojson::null>()) {
            setConditions(&scene.conditions, scene_.get<picojson::object>()["conditions"].get<picojson::object>());
        }
        if (!scene_.get<picojson::object>()["actions"].is<picojson::null>()) {
            setActions(&scene.actions, scene_.get<picojson::object>()["actions"].get<picojson::array>());
        }
        if (!scene_.get<picojson::object>()["next_scenes"].is<picojson::null>()) {
            setNextScenes(&scene.next_scenes, scene_.get<picojson::object>()["next_scenes"].get<picojson::array>());
        }
        (*scenes).push_back(scene);
    }
}

void setEndingScenes(vector<TScene> *ending_scenes, picojson::array ending_scenes_) {
    setScenes(ending_scenes, ending_scenes_);
}

void setScenario(TScenario *scenario, picojson::object scenario_) {
    setOpeningScene(&scenario->opening_scene, scenario_["opening_scene"].get<picojson::object>());
    if (!scenario_["scenes"].is<picojson::null>()) {
        setScenes(&scenario->scenes, scenario_["scenes"].get<picojson::array>());
    }
    setEndingScenes(&scenario->ending_scenes, scenario_["ending_scenes"].get<picojson::array>());
}

void setStopConditions(TConditions *stop_conditions, picojson::object stop_conditions_) {
    setConditions(stop_conditions, stop_conditions_);
}

int LoadScenarioJson(const string &path, TScenarioJson* scenario) {
    ifstream ifs(path, ios::in);
    if (ifs.fail()) {
        return 0;
    }
    const string json((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
    ifs.close();

    picojson::value v;
    const string err = parse(v, json);
    if (!err.empty()) {
        return 0;
    }

    picojson::object root = v.get<picojson::object>();

    scenario->map_id = root["map_id"].get<string>();

    setActors(&scenario->actors, root["actors"].get<picojson::object>());
    setScenario(&scenario->scenario, root["scenario"].get<picojson::object>());
    if (!root["stop_conditions"].is<picojson::null>()) {
        setStopConditions(&scenario->stop_conditions, root["stop_conditions"].get<picojson::object>());
    }

    cout << path << endl;

    return 1;
}