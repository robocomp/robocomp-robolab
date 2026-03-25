#include "urdf_mesh_loader.h"
#include <tinyxml2.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <regex>

using namespace tinyxml2;

URDFMeshLoader::URDFMeshLoader(RTCDevice& device, RTCScene& scene) 
    : m_device(device), m_scene(scene), m_root_link(nullptr) {}

URDFMeshLoader::~URDFMeshLoader() {
    for(auto& pair : m_links) delete pair.second;
    for(auto& pair : m_joints) delete pair.second;
}

Eigen::Matrix4f URDFMeshLoader::parseOrigin(void* xml_element) {
    XMLElement* origin_xml = static_cast<XMLElement*>(xml_element);
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    if(!origin_xml) return mat;
    
    const char* xyz_cstr = origin_xml->Attribute("xyz");
    const char* rpy_cstr = origin_xml->Attribute("rpy");
    
    if(xyz_cstr) {
        float x, y, z;
        if(sscanf(xyz_cstr, "%f %f %f", &x, &y, &z) == 3) {
            mat(0,3) = x; mat(1,3) = y; mat(2,3) = z;
        }
    }
    
    if(rpy_cstr) {
        float r, p, y;
        if(sscanf(rpy_cstr, "%f %f %f", &r, &p, &y) == 3) {
            Eigen::AngleAxisf rollAngle(r, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf pitchAngle(p, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf yawAngle(y, Eigen::Vector3f::UnitZ());
            Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
            mat.block<3,3>(0,0) = q.matrix();
        }
    }
    return mat;
}

Eigen::Vector3f URDFMeshLoader::parseAxis(void* xml_element) {
    XMLElement* axis_xml = static_cast<XMLElement*>(xml_element);
    Eigen::Vector3f axis(1, 0, 0);
    if(!axis_xml) return axis;
    
    const char* xyz_cstr = axis_xml->Attribute("xyz");
    if(xyz_cstr) {
        float x, y, z;
        if(sscanf(xyz_cstr, "%f %f %f", &x, &y, &z) == 3) {
            axis = Eigen::Vector3f(x, y, z).normalized();
        }
    }
    return axis;
}

bool URDFMeshLoader::parseLink(void* xml_element) {
    XMLElement* link_xml = static_cast<XMLElement*>(xml_element);
    const char* name = link_xml->Attribute("name");
    if(!name) return false;
    
    LinkNode* link = new LinkNode();
    link->name = name;
    
    XMLElement* geom_parent_xml = link_xml->FirstChildElement("collision");
    if(!geom_parent_xml) geom_parent_xml = link_xml->FirstChildElement("visual");
    
    if(geom_parent_xml) {
        XMLElement* geom_xml = geom_parent_xml->FirstChildElement("geometry");
        if(geom_xml) {
            XMLElement* mesh_xml = geom_xml->FirstChildElement("mesh");
            if(mesh_xml && mesh_xml->Attribute("filename")) {
                std::string filename = mesh_xml->Attribute("filename");
                if(filename.find("package://") == 0) {
                    filename = filename.substr(10);
                }
                size_t pos_meshes = filename.find("meshes/");
                if(pos_meshes != std::string::npos) {
                    filename = filename.substr(pos_meshes);
                }
                loadMeshIntoEmbree(link, m_base_dir + filename);
            }
        }
    }
    
    m_links[link->name] = link;
    return true;
}

bool URDFMeshLoader::parseJoint(void* xml_element) {
    XMLElement* joint_xml = static_cast<XMLElement*>(xml_element);
    const char* name = joint_xml->Attribute("name");
    const char* type = joint_xml->Attribute("type");
    if(!name || !type) return false;
    
    JointNode* joint = new JointNode();
    joint->name = name;
    joint->type = type;
    
    XMLElement* origin_xml = joint_xml->FirstChildElement("origin");
    joint->origin = parseOrigin(origin_xml);
    
    XMLElement* axis_xml = joint_xml->FirstChildElement("axis");
    joint->axis = parseAxis(axis_xml);
    
    XMLElement* parent_xml = joint_xml->FirstChildElement("parent");
    if(parent_xml && parent_xml->Attribute("link")) {
        std::string parent_name = parent_xml->Attribute("link");
        if(m_links.count(parent_name)) {
            joint->parent_link = m_links[parent_name];
        }
    }
    
    XMLElement* child_xml = joint_xml->FirstChildElement("child");
    if(child_xml && child_xml->Attribute("link")) {
        std::string child_name = child_xml->Attribute("link");
        if(m_links.count(child_name)) {
            joint->child_link = m_links[child_name];
        }
    }
    
    std::regex re(".*_joint_([0-9]+)$");
    std::smatch match;
    if(std::regex_search(joint->name, match, re)) {
        joint->kinova_id = std::stoi(match[1]); 
    }
    
    m_joints[joint->name] = joint;
    return true;
}

void URDFMeshLoader::loadMeshIntoEmbree(LinkNode* link, const std::string& mesh_path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(mesh_path, 
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
        
    if(!scene || !scene->HasMeshes()) {
        std::cerr << "URDFMeshLoader: Failed to load mesh " << mesh_path << std::endl;
        return;
    }
    
    RTCScene link_scene = rtcNewScene(m_device);
    
    for(unsigned int m=0; m < scene->mNumMeshes; ++m) {
        aiMesh* mesh = scene->mMeshes[m];
        RTCGeometry geom = rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_TRIANGLE);
        
        float* vertices = (float*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), mesh->mNumVertices);
        for(unsigned int i=0; i<mesh->mNumVertices; ++i) {
            vertices[i*3+0] = mesh->mVertices[i].x;
            vertices[i*3+1] = mesh->mVertices[i].y;
            vertices[i*3+2] = mesh->mVertices[i].z;
        }
        
        unsigned int* indices = (unsigned int*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned int), mesh->mNumFaces);
        for(unsigned int i=0; i<mesh->mNumFaces; ++i) {
            aiFace face = mesh->mFaces[i];
            if(face.mNumIndices != 3) continue;
            indices[i*3+0] = face.mIndices[0];
            indices[i*3+1] = face.mIndices[1];
            indices[i*3+2] = face.mIndices[2];
        }
        
        rtcCommitGeometry(geom);
        rtcAttachGeometry(link_scene, geom);
        rtcReleaseGeometry(geom);
    }
    
    rtcCommitScene(link_scene);
    
    RTCGeometry instance = rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_INSTANCE);
    rtcSetGeometryInstancedScene(instance, link_scene);
    rtcSetGeometryTimeStepCount(instance, 1);
    
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    rtcSetGeometryTransform(instance, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, I.data());
    
    rtcCommitGeometry(instance);
    
    link->embree_instance_id = rtcAttachGeometry(m_scene, instance);
    link->has_geometry = true;
    
    rtcReleaseGeometry(instance);
    rtcReleaseScene(link_scene);
    
    std::cout << "Loaded mesh for " << link->name << " (" << mesh_path << ")" << std::endl;
}

bool URDFMeshLoader::loadURDF(const std::string& urdf_path, const std::string& base_dir) {
    m_base_dir = base_dir;
    if (m_base_dir.back() != '/') m_base_dir += "/";

    XMLDocument doc;
    if (doc.LoadFile(urdf_path.c_str()) != XML_SUCCESS) {
        std::cerr << "URDFMeshLoader: Failed to load URDF " << urdf_path << std::endl;
        return false;
    }

    XMLElement* robot = doc.FirstChildElement("robot");
    if (!robot) return false;

    for (XMLElement* el = robot->FirstChildElement(); el != nullptr; el = el->NextSiblingElement()) {
        std::string tag = el->Name();
        if (tag == "link") {
            parseLink(el);
        } else if (tag == "joint") {
            parseJoint(el);
        }
    }

    std::map<LinkNode*, bool> has_parent;
    for (auto& pair : m_joints) {
        JointNode* j = pair.second;
        if(j->parent_link) j->parent_link->children.push_back(j);
        if(j->child_link) {
            j->child_link->parent_joint = j;
            has_parent[j->child_link] = true;
        }
    }

    for(auto& pair : m_links) {
        if(!has_parent[pair.second]) {
            m_root_link = pair.second;
            break;
        }
    }
    
    std::cout << "URDF Loaded successfully. Root link: " << (m_root_link ? m_root_link->name : "NONE") << std::endl;
    return true;
}

bool URDFMeshLoader::loadSingleSTL(const std::string& stl_path) {
    LinkNode* link = new LinkNode();
    link->name = "base_link";
    loadMeshIntoEmbree(link, stl_path);
    m_links[link->name] = link;
    m_root_link = link;
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    if(link->has_geometry) {
        RTCGeometry geom = rtcGetGeometry(m_scene, link->embree_instance_id);
        rtcSetGeometryTransform(geom, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, I.data());
        rtcCommitGeometry(geom);
        rtcCommitScene(m_scene);
    }
    return true;
}

void URDFMeshLoader::updateJoints(const std::map<int, float>& joint_angles) {
    if(!m_root_link) return;
    computeFK(m_root_link, Eigen::Matrix4f::Identity(), joint_angles);
    rtcCommitScene(m_scene);
}

void URDFMeshLoader::computeFK(LinkNode* link, const Eigen::Matrix4f& parent_transform, const std::map<int, float>& joint_angles) {
    Eigen::Matrix4f current_transform = parent_transform;

    if (link->parent_joint) {
        JointNode* j = link->parent_joint;
        Eigen::Matrix4f joint_transform = Eigen::Matrix4f::Identity();
        
        float angle = 0.0f;
        if(j->kinova_id != -1 && joint_angles.count(j->kinova_id)) {
            angle = joint_angles.at(j->kinova_id);
        } else if (j->kinova_id != -1 && joint_angles.count(j->kinova_id - 1)) {
            angle = joint_angles.at(j->kinova_id - 1);
        } else if (j->kinova_id != -1 && joint_angles.count(j->kinova_id + 1)) {
            // Because some ids might be shifted
            angle = joint_angles.at(j->kinova_id + 1);
        }
        
        if (j->type == "revolute" || j->type == "continuous") {
            Eigen::AngleAxisf aa(angle, j->axis);
            joint_transform.block<3,3>(0,0) = aa.toRotationMatrix();
        } else if (j->type == "prismatic") {
            joint_transform.block<3,1>(0,3) = j->axis * angle;
        }
        
        current_transform = parent_transform * j->origin * joint_transform;
    }

    if (link->has_geometry) {
        RTCGeometry geom = rtcGetGeometry(m_scene, link->embree_instance_id);
        rtcSetGeometryTransform(geom, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, current_transform.data());
        rtcCommitGeometry(geom);
    }

    for (JointNode* child_joint : link->children) {
        if (child_joint->child_link) {
            computeFK(child_joint->child_link, current_transform, joint_angles);
        }
    }
}
