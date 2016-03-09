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

#include "stampedinterval_visual.h"




typedef struct {
    double r;       // percent
    double g;       // percent
    double b;       // percent
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
} hsv;

static hsv   rgb2hsv(rgb in);
static rgb   hsv2rgb(hsv in);

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
            // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

namespace rviz_ia_plugins
{

StampedIntervalVisual::StampedIntervalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the StampedInterval's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

}

StampedIntervalVisual::~StampedIntervalVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void StampedIntervalVisual::setMessage( const ia_msgs::StampedInterval::ConstPtr& msg )
{

  for (auto itt = list_shape_.begin(); itt!=list_shape_.end(); ++itt)
  {
     for (auto it = (*itt).begin(); it!=(*itt).end(); ++it)
        (*it).reset();
  }
  list_shape_.clear();
  for (auto itt = msg->data.cbegin(); itt!=msg->data.cend(); ++itt)
  {
     std::vector<boost::shared_ptr<rviz::Shape> > tmpVector;
     for  (auto it = itt->data.cbegin(); it!=itt->data.cend(); ++it)
     {
       // Convert the geometry_msgs::position to an Ogre::Vector3.
       Ogre::Vector3 pose( (*it).position.x+(*it).width/2.0, (*it).position.y+(*it).height/2.0, 0);

       // Scale the arrow's thickness in each dimension along with its length.
       Ogre::Vector3 scale( (*it).width, (*it).height, 0 );
       boost::shared_ptr<rviz::Shape> shape_(new rviz::Shape(rviz::Shape::Type(1), scene_manager_, frame_node_ ));
       shape_->setScale( scale );
       shape_->setPosition( pose );
       tmpVector.push_back(shape_);
     }
     list_shape_.push_back(tmpVector);
  }
}

// Position and orientation are passed through to the SceneNode.
void StampedIntervalVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void StampedIntervalVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through.
void StampedIntervalVisual::setColor( float r, float g, float b, float a )
{
  rgb in;
  in.r = r;
  in.g = g;
  in.b = b;
  hsv  out = rgb2hsv(in);
  hsv  out2 = rgb2hsv(in);
  for (int i= 0; i<list_shape_.size(); i++)
  {
     out2.v = out.v/(1+(float)(i));
     rgb  intOut = hsv2rgb(out2);
     for (auto it = list_shape_[i].begin(); it!=list_shape_[i].end(); ++it)
        (*it)->setColor( intOut.r, intOut.g, intOut.b, a );
  }
}
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

