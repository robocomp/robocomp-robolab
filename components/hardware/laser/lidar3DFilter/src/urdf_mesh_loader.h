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
};

struct JointNode {
    std::string name;
    std::string type; // "revolute", "continuous", "prismatic", "fixed"
    Eigen::Vector3f axis;
    Eigen::Matrix4f origin; 
    
    LinkNode* parent_link = nullptr;
    LinkNode* child_link = nullptr;
    int kinova_id = -1; 
};

class URDFMeshLoader {
public:
    URDFMeshLoader(RTCDevice& device, RTCScene& scene);
    ~URDFMeshLoader();

    bool loadURDF(const std::string& urdf_path, const std::string& base_dir);
    bool loadSingleSTL(const std::string& stl_path);
    
    void updateJoints(const std::map<int, float>& joint_angles);

private:
    RTCDevice m_device;
    RTCScene m_scene;
    std::map<std::string, LinkNode*> m_links;
    std::map<std::string, JointNode*> m_joints;
    LinkNode* m_root_link = nullptr;
    std::string m_base_dir;

    bool parseLink(void* xml_element); // void* to avoid exposing tinyxml2 in header
    bool parseJoint(void* xml_element);
    void loadMeshIntoEmbree(LinkNode* link, const std::string& mesh_path);
    
    Eigen::Matrix4f parseOrigin(void* xml_element);
    Eigen::Vector3f parseAxis(void* xml_element);
    
    void computeFK(LinkNode* link, const Eigen::Matrix4f& parent_transform, const std::map<int, float>& joint_angles);
};

#endif
