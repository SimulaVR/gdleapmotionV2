#include "gdlm_sensor.h"
#include <iostream>
#include <thread>

#include "core/object.h"
#include "scene/main/node.h"

#include "core/array.h"
#include "servers/arvr_server.h"
#include "servers/arvr/arvr_positional_tracker.h"
#include "servers/arvr/arvr_interface.h"
#include "scene/3d/arvr_nodes.h"

#include "core/dictionary.h"
#include "core/global_constants.h"
#include "core/os/os.h"
#include "scene/resources/packed_scene.h"
#include "core/resource.h"
#include "core/io/resource_loader.h"
#include "scene/3d/skeleton.h"
#include "scene/3d/spatial.h"
#include "core/math/transform.h"

//#include <LeapC.h>
#include <Leap.h>

#define PI 3.14159265359f

// // void GDLMSensor::_register_methods() {
// // 	Dictionary args;

// // 	args[Variant("hand")] = Variant(Variant::OBJECT);
// // 	register_signal<GDLMSensor>("new_hand", args);
// // 	register_signal<GDLMSensor>("about_to_remove_hand", args);

// // 	register_method("get_is_running", &GDLMSensor::get_is_running);
// // 	register_method("get_is_connected", &GDLMSensor::get_is_connected);
// // 	register_method("get_left_hand_scene", &GDLMSensor::get_left_hand_scene);
// // 	register_method("set_left_hand_scene", &GDLMSensor::set_left_hand_scene);
// // 	register_method("get_right_hand_scene", &GDLMSensor::get_right_hand_scene);
// // 	register_method("set_right_hand_scene", &GDLMSensor::set_right_hand_scene);
// // 	register_method("get_arvr", &GDLMSensor::get_arvr);
// // 	register_method("set_arvr", &GDLMSensor::set_arvr);
// // 	register_method("get_smooth_factor", &GDLMSensor::get_smooth_factor);
// // 	register_method("set_smooth_factor", &GDLMSensor::set_smooth_factor);
// // 	register_method("get_keep_frames", &GDLMSensor::get_keep_frames);
// // 	register_method("set_keep_frames", &GDLMSensor::set_keep_frames);
// // 	register_method("get_keep_last_hand", &GDLMSensor::get_keep_last_hand);
// // 	register_method("set_keep_last_hand", &GDLMSensor::set_keep_last_hand);
// // 	register_method("get_hmd_to_leap_motion", &GDLMSensor::get_hmd_to_leap_motion);
// // 	register_method("set_hmd_to_leap_motion", &GDLMSensor::set_hmd_to_leap_motion);
// // 	register_method("_physics_process", &GDLMSensor::_physics_process);
// // 	register_method("get_finger_name", &GDLMSensor::get_finger_name);
// // 	register_method("get_finger_bone_name", &GDLMSensor::get_finger_bone_name);

// // 	register_property<GDLMSensor, bool>("arvr", &GDLMSensor::set_arvr, &GDLMSensor::get_arvr, false);
// // 	register_property<GDLMSensor, float>("smooth_factor", &GDLMSensor::set_smooth_factor, &GDLMSensor::get_smooth_factor, 0.5);
// // 	register_property<GDLMSensor, int>("keep_hands_for_frames", &GDLMSensor::set_keep_frames, &GDLMSensor::get_keep_frames, 60);
// // 	register_property<GDLMSensor, bool>("keep_last_hand", &GDLMSensor::set_keep_last_hand, &GDLMSensor::get_keep_last_hand, true);

// // 	register_property<GDLMSensor, String>("left_hand_scene", &GDLMSensor::set_left_hand_scene, &GDLMSensor::get_left_hand_scene, String());
// // 	register_property<GDLMSensor, String>("right_hand_scene", &GDLMSensor::set_right_hand_scene, &GDLMSensor::get_right_hand_scene, String());

// // 	// assume rotated by 90 degrees on x axis and -180 on Y and 8cm from center
// // 	Transform htlp;
// // 	htlp.basis = Basis(Vector3(90.0f * PI / 180.0f, -180.0f * PI / 180.0f, 0.0f));
// // 	htlp.origin = Vector3(0.0f, 0.0f, -0.08f);
// // 	register_property<GDLMSensor, Transform>("hmd_to_leap_motion", &GDLMSensor::set_hmd_to_leap_motion, &GDLMSensor::get_hmd_to_leap_motion, htlp);
// // }

void GDLMSensor::_init() {

}

GDLMSensor::GDLMSensor() {
	printf("Construct leap motion\n");

	is_running = false;
	is_connected = false;
	arvr = false;
	keep_last_hand = true;
	smooth_factor = 0.5;
	last_device = NULL;
	last_frame_id = 0;
	keep_hands_for_frames = 60;

	// assume rotated by 90 degrees on x axis and -180 on Y and 8cm from center
	hmd_to_leap_motion.basis = Basis(Vector3(90.0f * PI / 180.0f, -180.0f * PI / 180.0f, 0.0f));
	hmd_to_leap_motion.origin = Vector3(0.0f, 0.0f, -0.08f);

  while (!controller.isConnected()) {
    //printf("Waiting for controller to connect..\n");
  }
  controller.addListener(listener);
  set_is_running(true);
}

GDLMSensor::~GDLMSensor() {
	printf("Cleanup leap motion\n");

  // if our loop is still running, this will exit it...
  set_is_running(false);
  set_is_connected(false);

	if (last_device != NULL) {
		// free our device
		::free(last_device);
		last_device = NULL;
	}

	// and clear this JIC
	last_frame = Leap::Frame();

	// finally clean up hands, note that we don't need to free our scenes because they will be removed by Godot.
	while (hand_nodes.size() > 0) {
		GDLMSensor::hand_data *hd = hand_nodes.back();
		hand_nodes.pop_back();
		::free(hd);
	}
}

bool GDLMSensor::get_is_running() {
	bool ret;
	ret = is_running;
	return ret;
}

void GDLMSensor::set_is_running(bool p_set) {
	is_running = p_set;
	// maybe issue signal?
}

bool GDLMSensor::get_is_connected() {
	bool ret;
	ret = is_connected;
	return ret;
}

void GDLMSensor::set_is_connected(bool p_set) {
	if (is_connected != p_set) {
		is_connected = p_set;
		if (p_set) {

			if (arvr) {
				printf("Setting arvr to true\n");
        controller.setPolicy(Leap::Controller::POLICY_OPTIMIZE_HMD);
			} else {
				printf("Setting arvr to false\n");
        controller.clearPolicy(Leap::Controller::POLICY_OPTIMIZE_HMD);
			}
		}

		// maybe issue signal?
	}
}

bool GDLMSensor::wait_for_connection(int timeout, int waittime) {
	int time_passed = 0;
	while (!get_is_connected() && time_passed < timeout) {
		printf("Check...\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(waittime));
		time_passed += waittime;
	}

	return get_is_connected();
}

Leap::Frame GDLMSensor::get_last_frame() {
	Leap::Frame ret;
	ret = last_frame;
	return ret;
}

void GDLMSensor::set_last_frame(Leap::Frame p_frame) {
	last_frame = p_frame;
}

const Leap::Device *GDLMSensor::get_last_device() {
	const Leap::Device *ret;
	ret = last_device;
	return ret;
}

void GDLMSensor::set_last_device(const Leap::Device *p_device) {
	if (last_device != NULL) {
	} else {
		// allocate memory to store our device in
		last_device = (Leap::Device *)malloc(sizeof(*p_device));
	}

	// make a copy of our settings
	*last_device = *p_device;
}

bool GDLMSensor::get_arvr() const {
	return arvr;
}

void GDLMSensor::set_arvr(bool p_set) {
	if (arvr != p_set) {
		arvr = p_set;
		if (is_connected) {
			if (arvr) {
				printf("Setting arvr to true\n");
        controller.setPolicy(Leap::Controller::POLICY_OPTIMIZE_HMD);
			} else {
				printf("Setting arvr to false\n");
        controller.clearPolicy(Leap::Controller::POLICY_OPTIMIZE_HMD);
			}
		}
	}
}

float GDLMSensor::get_smooth_factor() const {
	return smooth_factor;
}

void GDLMSensor::set_smooth_factor(float p_smooth_factor) {
	smooth_factor = p_smooth_factor;
}

int GDLMSensor::get_keep_frames() const {
	return keep_hands_for_frames;
}

void GDLMSensor::set_keep_frames(int p_keep_frames) {
	keep_hands_for_frames = p_keep_frames;
}

bool GDLMSensor::get_keep_last_hand() const {
	return keep_last_hand;
}

void GDLMSensor::set_keep_last_hand(bool p_keep_hand) {
	keep_last_hand = p_keep_hand;
}

Transform GDLMSensor::get_hmd_to_leap_motion() const {
	return hmd_to_leap_motion;
}

void GDLMSensor::set_hmd_to_leap_motion(Transform p_transform) {
	hmd_to_leap_motion = p_transform;
}

String GDLMSensor::get_left_hand_scene() const {
	return hand_scene_names[0];
}

void GDLMSensor::set_left_hand_scene(String p_resource) {
	if (hand_scene_names[0] != p_resource) {
		hand_scene_names[0] = p_resource;

		// maybe delay loading until we need it?
		hand_scenes[0] = ResourceLoader::load(p_resource);
	}
}

String GDLMSensor::get_right_hand_scene() const {
	return hand_scene_names[1];
}

void GDLMSensor::set_right_hand_scene(String p_resource) {
	if (hand_scene_names[1] != p_resource) {
		hand_scene_names[1] = p_resource;

		// maybe delay loading until we need it?
		hand_scenes[1] = ResourceLoader::load(p_resource);
	}
}

const char *const GDLMSensor::finger[] = {
	"Thumb", "Index", "Middle", "Ring", "Pink"
};

String GDLMSensor::get_finger_name(int p_idx) {
	String finger_name;

	if (p_idx > 0 && p_idx <= 5) {
		finger_name = finger[p_idx - 1];
	}

	return finger_name;
}

const char *const GDLMSensor::finger_bone[] = {
	"Metacarpal", "Proximal", "Intermediate", "Distal"
};

String GDLMSensor::get_finger_bone_name(int p_idx) {
	String finger_bone_name;

	if (p_idx > 0 && p_idx <= 5) {
		finger_bone_name = finger_bone[p_idx - 1];
	}

	return finger_bone_name;
}

void GDLMSensor::update_hand_data(GDLMSensor::hand_data *p_hand_data, Leap::Hand *p_leap_hand) {
	Array args;

	if (p_hand_data == NULL)
		return;

	if (p_hand_data->scene == NULL)
		return;

	// first pinch distance
	//args.push_back(Variant(p_leap_hand->pinchDistance())); //pinchDistance() doesn't exist until V3 SDK
	args.push_back(Variant(p_leap_hand->pinchStrength())); //...so just use pinchStrength() again? :/
	//p_hand_data->scene->call("set_pinch_distance", args);
	p_hand_data->scene->call("set_pinch_distance", p_leap_hand->pinchStrength());

	// then pinch strength
	args.clear();
	args.push_back(Variant(p_leap_hand->pinchStrength()));
	//p_hand_data->scene->call("set_pinch_strength", args);
	p_hand_data->scene->call("set_pinch_strength", p_leap_hand->pinchStrength());

	// and grab strength
	args.clear();
	args.push_back(Variant(p_leap_hand->grabStrength()));
	//p_hand_data->scene->call("set_grab_strength", args);
	p_hand_data->scene->call("set_grab_strength", p_leap_hand->grabStrength());

  //printf("pinchStrength(): %f\n", p_leap_hand->pinchStrength());
	if(p_leap_hand->pinchStrength() > 0.9) {
			if (p_hand_data->type == 0) {
				is_pinched_left = true;
				last_pinched_frame = controller.frame();
			} else if (p_hand_data->type == 1) {
				is_pinched_right = true;
				last_pinched_frame = controller.frame();
			}
	} else {
	  if (p_hand_data->type == 0) {
		  is_pinched_left = false;
		} else if (p_hand_data->type == 1) {
		  is_pinched_right = false;
	  }
 }

  //printf("grabStrength(): %f\n", p_leap_hand->grabStrength());
	if(p_leap_hand->grabStrength() > 0.9) {
		if (p_hand_data->type == 0) {
			is_grabbed_left = true;
			last_grabbed_frame = controller.frame();
		} else if (p_hand_data->type == 1) {
			is_grabbed_right = true;
			last_grabbed_frame = controller.frame();
		}
	} else {
	  if (p_hand_data->type == 0) {
		  is_grabbed_left = false;
		} else if (p_hand_data->type == 1) {
		  is_grabbed_right = false;
	  }
	}
};

void GDLMSensor::update_hand_position(GDLMSensor::hand_data *p_hand_data, Leap::Hand *p_leap_hand) {
	Transform hand_transform;

	if (p_hand_data == NULL)
		return;

	if (p_hand_data->scene == NULL)
		return;

  Leap::Matrix mat = p_leap_hand->basis();
  Basis base_orientation( Vector3(mat.xBasis.x, mat.xBasis.y, mat.xBasis.z)
                        , Vector3(mat.yBasis.x, mat.yBasis.y, mat.yBasis.z)
                        , Vector3(mat.zBasis.x, mat.zBasis.y, mat.zBasis.z));

	// apply to our transform
	hand_transform.set_basis(base_orientation);

	// position of our hand
  Leap::Vector palmPosition = p_leap_hand->palmPosition();
  Vector3 hand_position(
      palmPosition.x * world_scale,
      palmPosition.y * world_scale,
      palmPosition.z * world_scale);
	hand_transform.set_origin(hand_position);

	// get our inverse for positioning the rest of the hand
	Transform hand_inverse = hand_transform.inverse();

	// if in ARVR mode we should xform this to convert from HMD relative position to Origin world position
	if (arvr) {
		Transform last_transform = p_hand_data->scene->get_transform();
		hand_transform = hmd_transform * hmd_to_leap_motion * hand_transform;

		// leap motions frame interpolation is pretty good but we're going to smooth things out a little bit
		// to stop hands from visible drifting when the user turns his/her head. We can live with the position
		// of the hand being a few frames behind
		hand_transform.origin = last_transform.origin.linear_interpolate(hand_transform.origin, smooth_factor);
	};

	// and apply
	p_hand_data->scene->set_transform(hand_transform);

	// lets parse our digits
	for (int d = 0; d < 5; d++) {
		Leap::Finger digit = p_leap_hand->fingers()[d];
    Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(0);
		Leap::Bone bone = digit.bone(boneType);

		// logic for positioning stuff
		Transform parent_inverse = hand_inverse;
		Vector3 up = Vector3(0.0, 1.0, 0.0);
		Transform bone_pose;

		// Our first bone provides our starting position for our first node
    Leap::Vector prevJoint = bone.prevJoint();
		Vector3 bone_start_pos(
				prevJoint.x * world_scale,
				prevJoint.y * world_scale,
				prevJoint.z * world_scale);
		bone_start_pos = parent_inverse.xform(bone_start_pos);
		bone_pose.origin = bone_start_pos;

		// For now assume order, we may change this to naming, we should cache this, maybe do this when we load our scene.
		Spatial *digit_node = p_hand_data->finger_nodes[d];
		if (digit_node != NULL) {
			// And handle our bones.
			int first_bone = d == 0 ? 1 : 0; // we skip the first bone for our thumb
			for (int b = first_bone; b < 4; b++) {
        Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(b);
        bone = digit.bone(boneType);

				// We calculate rotation based on our next joint position
        Leap::Vector nextJoint = bone.nextJoint();
				Vector3 bone_pos(
						nextJoint.x * world_scale,
						nextJoint.y * world_scale,
						nextJoint.z * world_scale);
				// transform it based on our last locale
				bone_pos = parent_inverse.xform(bone_pos);
				// remove previous position to get our delta
				bone_pos -= bone_pose.origin;

				// Standard cross normalise with up to create our matrix. Could fail on a 90 degree bend.
				Vector3 axis_z = bone_pos;
				axis_z.normalize();
				Vector3 axis_x = up.cross(axis_z).normalized();
				Vector3 axis_y = axis_z.cross(axis_x).normalized();

				bone_pose.basis.set_axis(0, axis_x);
				bone_pose.basis.set_axis(1, axis_y);
				bone_pose.basis.set_axis(2, axis_z);

				if (digit_node != NULL) {
					// now we can set our parent
					digit_node->set_transform(bone_pose);

					digit_node = p_hand_data->digit_nodes[d][b];
				}

				// and update for next iteration
				parent_inverse = bone_pose.inverse() * parent_inverse;

				// Our next nodes origin...
				bone_pose.origin = Vector3(0.0, 0.0, bone_pos.length());
			}

			// set our last digits transform...
			if (digit_node != NULL) {
				bone_pose.basis = Basis();
				digit_node->set_transform(bone_pose);
			}
		}
	}

	// do we want to do something with the arm?
}

GDLMSensor::hand_data *GDLMSensor::find_hand_by_id(int p_type, uint32_t p_leap_id) {
	for (int h = 0; h < hand_nodes.size(); h++) {
		if ((hand_nodes[h]->type == p_type) && (hand_nodes[h]->leap_id == p_leap_id)) {
			return hand_nodes[h];
		}
	}

	return NULL;
}

GDLMSensor::hand_data *GDLMSensor::find_unused_hand(int p_type) {
	for (int h = 0; h < hand_nodes.size(); h++) {
		// note, unused_frames must be bigger then 0, else its just that we've reset this.
		if ((hand_nodes[h]->type == p_type) && (hand_nodes[h]->active_this_frame == false) && (hand_nodes[h]->unused_frames > 0)) {
			return hand_nodes[h];
		}
	}

	return NULL;
}

int GDLMSensor::count_hands(int p_type, bool p_active_only) {
	int count = 0;
	for (int h = 0; h < hand_nodes.size(); h++) {
		if (hand_nodes[h]->type == p_type && (hand_nodes[h]->active_this_frame || !p_active_only)) {
			count++;
		}
	}

	return count;
}

GDLMSensor::hand_data *GDLMSensor::new_hand(int p_type, uint32_t p_leap_id) {

	printf("GDLMSensor::new_hand(..)");
	if (hand_scenes[p_type].is_null()) {
	  printf("GDLMSensor::new_hand(..) 1");
		return NULL;
	} else if (!hand_scenes[p_type]->can_instance()) {
	  printf("GDLMSensor::new_hand(..) 2");
		return NULL;
	}

	hand_data *new_hand_data = (hand_data *)malloc(sizeof(hand_data));

	new_hand_data->type = p_type;
	new_hand_data->leap_id = p_leap_id;
	new_hand_data->active_this_frame = true;
	new_hand_data->unused_frames = 0;

	new_hand_data->scene = (Spatial *)hand_scenes[p_type]->instance(); // is it safe to cast like this?
	new_hand_data->scene->set_name(String("Hand ") + String(std::to_string(p_type).c_str()) + String(" ") + String(std::to_string(p_leap_id).c_str()));
	add_child(new_hand_data->scene, false);

	for (int d = 0; d < 5; d++) {
		Spatial *node = (Spatial *)new_hand_data->scene->find_node(String(finger[d]), false);
		if (node == NULL) {
			printf("Couldn''t find node %s\n", finger[d]);

			// clear just in case
			new_hand_data->finger_nodes[d] = NULL;
			for (int b = 0; b < 4; b++) {
				new_hand_data->digit_nodes[d][b] = NULL;
			}
		} else {
			new_hand_data->finger_nodes[d] = node;

			int first_bone = 0;
			if (d == 0) {
				// we're one digit short on our thumb...
				new_hand_data->digit_nodes[0][0] = NULL;
				first_bone = 1;
			}
			for (int b = first_bone; b < 4; b++) {
				if (node != NULL) {
					// find our child node...
					char node_name[256];
					sprintf(node_name, "%s_%s", finger[d], finger_bone[b]);
					node = (Spatial *)node->find_node(String(node_name), false);
					if (node == NULL) {
						printf("Couldn''t find node %s\n", node_name);
					}
				}

				// even if node is NULL, assign it, we want to make sure we don't have old pointers...
				new_hand_data->digit_nodes[d][b] = node;
			}
		}
	}

	//Array args;
	//args.push_back(Variant(new_hand_data->scene));
	//args.push_back(new_hand_data->type);
	printf("new_hand signal emitted from gdlm_sensor.cpp");
	emit_signal("new_hand", new_hand_data->scene, new_hand_data->type);

	return new_hand_data;
}

void GDLMSensor::delete_hand(GDLMSensor::hand_data *p_hand_data) {
	printf("GDLMSensor::delete_hand(..)");
	// this should free everything up and invalidate it, no need to do anything more...
	if (p_hand_data->scene != NULL) {
		//Array args;
		//args.push_back(Variant(p_hand_data->scene));
		emit_signal("about_to_remove_hand", p_hand_data->scene);

		// hide and then queue free, this will properly destruct our scene and remove it from our tree
		p_hand_data->scene->hide();
		p_hand_data->scene->queue_delete(); // since `ClassDB::bind_method(D_METHOD("queue_free"), &Node::queue_delete);`
	}

	::free(p_hand_data);
}

// our Godot physics process, runs within the physic thread and is responsible for updating physics related stuff
void GDLMSensor::_physics_process_cpp(float delta) {
	//printf("GDLMSensor::_physics_process_cpp(..)");
  Leap::Frame frame;
	uint64_t arvr_frame_usec;

	// We're getting our measurements in mm, want them in m
	world_scale = 0.001f;

	// get some arvr stuff
	if (arvr) {
		// At this point in time our timing is such that last_process_usec + last_frame_usec = last_commit_usec
		// and it will be the frame previous to the one we're rendering and it will be the frame get_hmd_transform relates to.
		// Once we start running rendering in a separate thread last_commit_usec will still be the last frame
		// but last_process_usec will be newer and get_hmd_transform could be more up to date.
		// last_frame_usec will be an average.
		// We probably should put this whole thing into a mutex with the render thread once the time is right.
		// For now however.... :)

		ARVRServer *arvr_server = ARVRServer::get_singleton();

		world_scale *= arvr_server->get_world_scale();
		hmd_transform = arvr_server->get_hmd_transform();
		arvr_frame_usec = arvr_server->get_last_process_usec() + arvr_server->get_last_frame_usec();
	}

	// update our timing
		uint64_t godot_usec = OS::get_singleton()->get_ticks_msec() * 1000; // why does godot not give us usec while it records it, grmbl...

	// get our frame, either interpolated or latest
	if (arvr && arvr_frame_usec != 0) {
    frame = controller.frame();
    last_frame = controller.frame(); //hack

	} else {
		// ok lets process our last frame. Note that leap motion says it caches these so I'm assuming they
		// are valid for a few frames.
		frame = get_last_frame();
	}

  frame = controller.frame(); //hack

	// was everything above successful?
  if (!arvr && (last_frame_id == frame.id())) {
		// we already parsed this, no need to do this. In ARVR we may need to do more
		return;
	}

	last_frame_id = frame.id();

	// Lets process our frames...

	// Mark all current hand nodes as inactive, we'll mark the ones that are active as we find they are still used
	for (int h = 0; h < hand_nodes.size(); h++) {
		// if its already inactive we don't want to reset unused frames.
		if (hand_nodes[h]->active_this_frame) {
			hand_nodes[h]->active_this_frame = false;
			hand_nodes[h]->unused_frames = 0;
		}
	}

	// process the hands we're getting from leap motion
	for (uint32_t h = 0; h < frame.hands().count(); h++) {
		Leap::Hand hand = frame.hands()[h];
		int type = hand.isLeft() ? 0 : 1;

		// see if we already have a scene for this hand
		hand_data *hd = find_hand_by_id(type, hand.id());
		if (hd == NULL) {
			// nope? then see if we can find a hand we lost tracking for
			hd = find_unused_hand(type);
		}
		if (hd == NULL) {
			// nope? time to get a new hand
			hd = new_hand(type, hand.id());
			if (hd != NULL) {
				hand_nodes.push_back(hd);
				//emit_signal("new_hand")
			}
		}
		if (hd != NULL) {
			// yeah! mark as used and relate to our hand
			hd->active_this_frame = true;
			hd->unused_frames = 0;
			hd->leap_id = hand.id();

			// and update
			update_hand_data(hd, &hand);
			update_hand_position(hd, &hand);

			// should make sure hand is visible
		}
	}

	// and clean up, in reverse because we may remove entries
	for (int h = hand_nodes.size() - 1; h >= 0; h--) {
		hand_data *hd = hand_nodes[h];

		// not active?
		if (!hd->active_this_frame) {
			hd->unused_frames++;

			// lost tracking for awhile now? remove it unless its the last one
			if (hd->unused_frames > keep_hands_for_frames && (count_hands(hd->type) > 1 || !keep_last_hand)) {
				delete_hand(hd);
				hand_nodes.erase(hand_nodes.begin() + h);
			} else {
				// should make sure hand is invisible
			}
		}
	}
}

void GDLMSensor::_notification(int p_what) {
	switch (p_what) {
  case NOTIFICATION_PHYSICS_PROCESS:
		{
    float time = get_physics_process_delta_time();
    _physics_process_cpp(time);
    break;
		}
  default:
    return;
	}
}

bool GDLMSensor::get_is_pinched_left() {
	return is_pinched_left;
}

bool GDLMSensor::get_is_pinched_right() {
	return is_pinched_right;
}

bool GDLMSensor::get_is_grabbed_left() {
	return is_grabbed_left;
}

bool GDLMSensor::get_is_grabbed_right() {
	return is_grabbed_right;
}

float GDLMSensor::get_hands_scale_factor() {
  return last_frame.scaleFactor(last_pinched_frame);
}

void GDLMSensor::_bind_methods() {
  ClassDB::bind_method(D_METHOD("get_is_running"), &GDLMSensor::get_is_running);
  ClassDB::bind_method(D_METHOD("get_is_connected"), &GDLMSensor::get_is_connected);
  ClassDB::bind_method(D_METHOD("get_left_hand_scene"), &GDLMSensor::get_left_hand_scene);
  ClassDB::bind_method(D_METHOD("set_left_hand_scene", "left_hand_scene"), &GDLMSensor::set_left_hand_scene);
  ClassDB::bind_method(D_METHOD("get_right_hand_scene"), &GDLMSensor::get_right_hand_scene);
  ClassDB::bind_method(D_METHOD("set_right_hand_scene", "right_hand_scene"), &GDLMSensor::set_right_hand_scene);
  ClassDB::bind_method(D_METHOD("get_arvr"), &GDLMSensor::get_arvr);
  ClassDB::bind_method(D_METHOD("set_arvr", "arvr"), &GDLMSensor::set_arvr);
  ClassDB::bind_method(D_METHOD("get_smooth_factor"), &GDLMSensor::get_smooth_factor);
  ClassDB::bind_method(D_METHOD("set_smooth_factor", "smooth_factor"), &GDLMSensor::set_smooth_factor);
  ClassDB::bind_method(D_METHOD("get_keep_frames"), &GDLMSensor::get_keep_frames);
  ClassDB::bind_method(D_METHOD("set_keep_frames", "keep_frames"), &GDLMSensor::set_keep_frames);
  ClassDB::bind_method(D_METHOD("get_keep_last_hand"), &GDLMSensor::get_keep_last_hand);
  ClassDB::bind_method(D_METHOD("set_keep_last_hand", "keep_last_hand"), &GDLMSensor::set_keep_last_hand);
  ClassDB::bind_method(D_METHOD("get_hmd_to_leap_motion"), &GDLMSensor::get_hmd_to_leap_motion);
  ClassDB::bind_method(D_METHOD("set_hmd_to_leap_motion", "hmd_to_leap_motion"), &GDLMSensor::set_hmd_to_leap_motion);
  //ClassDB::bind_method(D_METHOD("_physics_process"), &GDLMSensor::_physics_process);
  ClassDB::bind_method(D_METHOD("get_finger_name"), &GDLMSensor::get_finger_name);
  ClassDB::bind_method(D_METHOD("get_finger_bone_name"), &GDLMSensor::get_finger_bone_name);
  ClassDB::bind_method(D_METHOD("get_is_pinched_left"), &GDLMSensor::get_is_pinched_left);
  ClassDB::bind_method(D_METHOD("get_is_pinched_right"), &GDLMSensor::get_is_pinched_right);
  ClassDB::bind_method(D_METHOD("get_is_grabbed_left"), &GDLMSensor::get_is_grabbed_left);
  ClassDB::bind_method(D_METHOD("get_is_grabbed_right"), &GDLMSensor::get_is_grabbed_right);
  ClassDB::bind_method(D_METHOD("get_hands_scale_factor"), &GDLMSensor::get_hands_scale_factor);


	ADD_SIGNAL(MethodInfo("new_hand", PropertyInfo(Variant::OBJECT, "hand", PROPERTY_HINT_RESOURCE_TYPE, "SceneTree")
	                                , PropertyInfo(Variant::INT, "hand_type")));
	ADD_SIGNAL(MethodInfo("about_to_remove_hand", PropertyInfo(Variant::OBJECT, "hand", PROPERTY_HINT_RESOURCE_TYPE, "SceneTree")));


	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "arvr",
				PROPERTY_HINT_PLACEHOLDER_TEXT, "arvr",
				PROPERTY_USAGE_DEFAULT_INTL),
			"set_arvr", "get_arvr");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "smooth_factor",
				PROPERTY_HINT_PLACEHOLDER_TEXT, "smooth_factor",
				PROPERTY_USAGE_DEFAULT_INTL),
			"set_smooth_factor", "get_smooth_factor");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "keep_hands_for_frames",
				PROPERTY_HINT_PLACEHOLDER_TEXT, "keep_hands_for_frames",
				PROPERTY_USAGE_DEFAULT_INTL),
			"set_keep_frames", "get_keep_frames");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "keep_last_hand",
				PROPERTY_HINT_PLACEHOLDER_TEXT, "keep_last_hand",
				PROPERTY_USAGE_DEFAULT_INTL),
			"set_keep_last_hand", "get_keep_last_hand");

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "left_hand_scene",
				PROPERTY_HINT_PLACEHOLDER_TEXT, "left_hand_scene",
				PROPERTY_USAGE_DEFAULT_INTL),
			"set_left_hand_scene", "get_left_hand_scene");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "right_hand_scene",
				PROPERTY_HINT_PLACEHOLDER_TEXT, "right_hand_scene",
				PROPERTY_USAGE_DEFAULT_INTL),
			"set_right_hand_scene", "get_right_hand_scene");
}


void GDLMListener::onConnect(const Leap::Controller &controller) {
  	printf("LeapMotion - connected to leap motion\n");
  	// update our status
    // GDLMSensor *sensor = (GDLMSensor *)controller;
  	// sensor->set_is_connected(true);
}

void GDLMListener::onDisconnect(const Leap::Controller &) {
	// update our status
	//set_is_connected(false);

	// log...
	printf("LeapMotion - onDisconnect\n");
}

void GDLMListener::onDeviceChange(const Leap::Controller &controller) {
  printf("Opening new device.\n");

  // GDLMSensor *sensor = (GDLMSensor *)controller;
  // sensor->set_last_device(controller->devices()[0]);
}

void GDLMListener::onDeviceFailure(const Leap::Controller &) {
  printf("Device has failed.\n");
}

// void GDLMListener::onDisconnect(const Leap::Controller &) {
// 	// just log for now
// 	printf("LeapMotion - Leap Motion daemon/service has been disconnected from your application.\n");
// }

void GDLMListener::onFrame(const Leap::Controller &controller) {
/*
	printf("LeapMotion - onFrame\n");
  // Leap::Frame frame = controller->frame();
  // GDLMSensor *sensor = (GDLMSensor *)controller;
  // sensor->set_last_frame(&frame);
  // Get the most recent frame and report some basic information
  const Leap::Frame frame = controller.frame();
  std::cout << "Frame id: " << frame.id()
            << ", timestamp: " << frame.timestamp()
            << ", hands: " << frame.hands().count()
            << ", extended fingers: " << frame.fingers().extended().count()
            << ", tools: " << frame.tools().count()
            << ", gestures: " << frame.gestures().count() << std::endl;
            */

}


// void GDLMListener::onLogMessage(const Leap::Controller &, Leap::MessageSeverity severity, int64_t timestamp, const char *msg) {
// 	printf("LeapMotion - onLogMessage\n");
// }

void GDLMListener::onExit(const Leap::Controller &) {
	printf("LeapMotion - onExit\n");
}
void GDLMListener::onFocusGained(const Leap::Controller &) {
	printf("LeapMotion - onFocusGained\n");
}
void GDLMListener::onFocusLost(const Leap::Controller &) {
	printf("LeapMotion - onFocusLost\n");
}
void GDLMListener::onImages(const Leap::Controller &) {
	printf("LeapMotion - onImages\n");
}
void GDLMListener::onInit(const Leap::Controller &) {
	printf("LeapMotion - onInit\n");
}
void GDLMListener::onServiceChange(const Leap::Controller &) {
	printf("LeapMotion - onServiceChange\n");
}
void GDLMListener::onServiceConnect(const Leap::Controller &) {
	printf("LeapMotion - onServiceConnect\n");
}
void GDLMListener::onServiceDisconnect(const Leap::Controller &) {
	printf("LeapMotion - onServiceDisconnect\n");
}
