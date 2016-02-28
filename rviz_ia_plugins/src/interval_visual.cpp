/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include "interval_visual.h"

namespace rviz_ia_plugins
{

IntervalVisual::IntervalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Interval's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

}

IntervalVisual::~IntervalVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void IntervalVisual::setMessage( const ia_msgs::Interval::ConstPtr& msg )
{
  for (auto it = list_shape_.begin(); it!=list_shape_.end(); ++it) {
     (*it).reset();
  }
  list_shape_.clear();
  for (auto it = msg->data.cbegin(); it!=msg->data.cend(); ++it) {
    // Convert the geometry_msgs::position to an Ogre::Vector3.
    Ogre::Vector3 pose( (*it).position.x+(*it).width/2.0, (*it).position.y+(*it).height/2.0, (*it).position.z );

    // Scale the arrow's thickness in each dimension along with its length.
    Ogre::Vector3 scale( (*it).width, (*it).height, 0 );
    boost::shared_ptr<rviz::Shape> shape_(new rviz::Shape(rviz::Shape::Type(1), scene_manager_, frame_node_ ));
    shape_->setScale( scale );
    shape_->setPosition( pose );
    list_shape_.push_back(shape_);
  }
}

// Position and orientation are passed through to the SceneNode.
void IntervalVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void IntervalVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through.
void IntervalVisual::setColor( float r, float g, float b, float a )
{
  for (auto it = list_shape_.cbegin(); it!=list_shape_.cend(); ++it) {
     (*it)->setColor( r, g, b, a );
  }
}
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

