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

#ifndef StampedInterval_VISUAL_H
#define StampedInterval_VISUAL_H

#include <ia_msgs/StampedInterval.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape;
}

namespace rviz_ia_plugins
{

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of StampedIntervalVisual represents the visualization of a single
// ia_msgs::StampedInterval message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class StampedIntervalVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  StampedIntervalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~StampedIntervalVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const ia_msgs::StampedInterval::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way StampedIntervalVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the StampedInterval message.
  void setColor( float r, float g, float b, float a );

private:
  std::vector<std::vector<boost::shared_ptr<rviz::Shape> > > list_shape_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the StampedInterval message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // StampedInterval_VISUAL_H
