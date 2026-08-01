#ifndef URDF_MESH_LOADER_H
#define URDF_MESH_LOADER_H

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <embree4/rtcore.h>

struct JointNode;

struct LinkNode {
    std::string name;
    std::vector<JointNode*> children;
    JointNode* parent_joint = nullptr;
    
    unsigned int embree_instance_id = -1;
    bool has_geometry = false;
    Eigen::Matrix4f visual_origin = Eigen::Matrix4f::Identity();
};

struct JointNode {
    std::string name;
    std::string type; // "revolute", "continuous", "prismatic", "fixed"
    Eigen::Vector3f axis;
    Eigen::Matrix4f origin; 
    
    LinkNode* parent_link = nullptr;
    LinkNode* child_link = nullptr;
};

class URDFMeshLoader {
public:
    URDFMeshLoader(RTCDevice& device, RTCScene& scene);
    ~URDFMeshLoader();

    bool loadURDF(const std::string& urdf_path, const std::string& base_dir);
    bool loadSingleSTL(const std::string& stl_path);
    // As loadSingleSTL but for ANY mesh Assimp can read (STL, OBJ, ...), with a rigid placement
    // applied as the Embree INSTANCE transform. The transform exists because a mesh authored for
    // display need not sit in the robot frame: shadow.obj's z-min is -0.050 m, i.e. it hangs below
    // the floor plane, and the project's own viewers recentre it before use. Getting that placement
    // wrong is the same class of bug as the 45 mm helios mount error — it silently moves the mesh by
    // a distance comparable to the 0.05 m query radius and the self-filter starts missing returns.
    bool loadSingleMesh(const std::string& mesh_path, const Eigen::Matrix4f& placement);
    
    void updateJoints(const std::map<std::string, float>& joint_angles);

private:
    RTCDevice m_device;
    RTCScene m_scene;
    std::map<std::string, LinkNode*> m_links;
    std::map<std::string, JointNode*> m_joints;
    LinkNode* m_root_link = nullptr;
    std::string m_base_dir;

    bool parseLink(void* xml_element); // void* to avoid exposing tinyxml2 in header
    bool parseJoint(void* xml_element);
    // Returns FALSE when the mesh cannot be read. It used to be `void`, and loadSingleMesh returned
    // true regardless — so a typo'd path produced an EMPTY Embree scene, a self-filter that dropped
    // nothing, and a phantom obstacle glued to the robot. Hours were lost to that in 2026-08.
    bool loadMeshIntoEmbree(LinkNode* link, const std::string& mesh_path);
    
    Eigen::Matrix4f parseOrigin(void* xml_element);
    Eigen::Vector3f parseAxis(void* xml_element);
    
    void computeFK(LinkNode* link, const Eigen::Matrix4f& parent_transform, const std::map<std::string, float>& joint_angles);
};

#endif
