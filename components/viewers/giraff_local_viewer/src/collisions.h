//
// Created by robolab on 13/01/20.
//

#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <innermodel/innermodel.h>
#include "CommonBehavior.h"


class Collisions {

public:
    std::vector<QString> robotNodes;
    std::vector<QString> restNodes;
    std::set<QString> excludedNodes;
    QRectF outerRegion;

    void initialize(std::shared_ptr<InnerModel> innerModel_, std::shared_ptr< RoboCompCommonBehavior::ParameterList > params_) {


        qDebug()<<"Collisions - " <<__FUNCTION__;
        innerModel = innerModel_;

        /// Processing configuration parameters
        try
        {
            outerRegion.setLeft(std::stof(params_->at("OuterRegionLeft").value));
            outerRegion.setRight(std::stof(params_->at("OuterRegionRight").value));
            outerRegion.setBottom(std::stof(params_->at("OuterRegionBottom").value));
            outerRegion.setTop(std::stof(params_->at("OuterRegionTop").value));
        }
        catch(const std::exception &e)
        {
            std::cout << "Exception " << e.what() << " Collisions::initialize(). OuterRegion parameters not found in config file" << std::endl;
            //robocomp::exception ex("OuterRegion parameters not found in config file");
            throw e;
        }

        if(outerRegion.isNull())
        {
            qDebug()<<"[ERROR] OUTER REGION IS NULL";
        }

        QStringList ls = QString::fromStdString(params_->at("ExcludedObjectsInCollisionCheck").value).replace(" ", "" ).split(',');
        qDebug() << __FILE__ << __FUNCTION__ << ls.size() << "objects read for exclusion list";

                foreach(const QString &s, ls)
                    excludedNodes.insert(s);

        // Compute the list of meshes that correspond to robot, world and possibly some additionally excluded ones
        robotNodes.clear(); restNodes.clear();
        recursiveIncludeMeshes(innerModel->getRoot(), "robot", false, robotNodes, restNodes, excludedNodes);
        qsrand( QTime::currentTime().msec() );

    }

    bool checkRobotValidStateAtTargetFast(const QVec &targetPos, const QVec &targetRot) const   {
        //First we move the robot in our copy of innermodel to its current coordinates
            innerModel->updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());
            ///////////////////////
            //// Check if the robot at the target collides with any know object
            ///////////////////////

            bool collision = false;

            for ( auto &in : robotNodes )
            {
                for ( auto &out : restNodes )
                {
                    try
                    {
                        collision = innerModel->collide(in, out);
                    }

                    catch(QString s) {qDebug()<< __FUNCTION__ << s;}

                    if (collision)
                    {
                        return false;
                    }
                }
            }
            return true;
        }


    void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out, std::set<QString> &excluded) {

        if (node->id == robotId)
        {
            inside = true;
        }

        InnerModelMesh *mesh;
        InnerModelPlane *plane;
        InnerModelTransform *transformation;

        if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
        {
            for (int i=0; i<node->children.size(); i++)
            {
                recursiveIncludeMeshes(node->children[i], robotId, inside, in, out, excluded);

            }

        }

        else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
        {
            if( std::find(excluded.begin(), excluded.end(), node->id) == excluded.end() )
            {
                if (inside)
                {
                    in.push_back(node->id);
                }
                else
                if(mesh or plane)
                    out.push_back(node->id);
            }
        }

    }

private:

    std::shared_ptr<InnerModel> innerModel;

};

#endif //COLLISIONS_H
