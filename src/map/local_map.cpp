#include <iostream>
#include <GL/gl.h>

#include "local_map.h"
#include "globals/opengl_primitives.h"

namespace nicp {

  using namespace std;

  LocalMap::LocalMap(const Eigen::Isometry3f& transform, 
		     int id,
		     nicp::IdContext* context) : MapNode(transform, id, context){
  }

  LocalMap::~LocalMap(){
    _cloud_ref.set(0);
  }
  
  void LocalMap::serialize(nicp::ObjectData& data, nicp::IdContext& context) {
    MapNode::serialize(data,context);
    nicp::ObjectData * blobData=new nicp::ObjectData();
    data.setField("cloud", blobData);
    _cloud_ref.serialize(*blobData,context);

    nicp::ArrayData* nodesArray = new nicp::ArrayData;
    for (MapNodeList::iterator it = _nodes.begin(); it!=_nodes.end(); it++){
      MapNode* node=it->get();
      nodesArray->add(new nicp::PointerData(node));
    }
    data.setField("nodes", nodesArray);

    nicp::ArrayData* relationsArray = new nicp::ArrayData;
    for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++){
      BinaryNodeRelation* rel=it->get();
      relationsArray->add(new nicp::PointerData(rel));
    }
    data.setField("relations", relationsArray);

  }
  
  void LocalMap::deserialize(nicp::ObjectData& data, nicp::IdContext& context) {
    MapNode::deserialize(data,context);

    // deserialize the cloud
    nicp::ObjectData * blobData = static_cast<nicp::ObjectData *>(data.getField("cloud"));
    _cloud_ref.deserialize(*blobData,context);

    nicp::ArrayData& nodesArray=data.getField("nodes")->getArray();    
    for (size_t i =0; i< nodesArray.size(); i++){
      nicp::ValueData& v = nodesArray[i];
      nicp::Identifiable* id = v.getPointer();
      MapNode* n = dynamic_cast<MapNode*>(id);
      _nodes.addElement(n);
    }
    
    nicp::ArrayData& relationsArray=data.getField("relations")->getArray();
    for (size_t i =0; i< relationsArray.size(); i++){
      nicp::ValueData& v = relationsArray[i];
      nicp::Identifiable* id = v.getPointer();
      BinaryNodeRelation* r = dynamic_cast<BinaryNodeRelation*>(id);
    }
  }

  void LocalMap::push() { 
    MapNode::push();
    for(MapNodeList::iterator nodes_it = _nodes.begin(); nodes_it != _nodes.end(); ++nodes_it) { 
      (*nodes_it)->push(); 
    }
  }

  void LocalMap::pop() { 
    MapNode::pop();
    for(MapNodeList::iterator nodes_it = _nodes.begin(); nodes_it != _nodes.end(); ++nodes_it) { 
      (*nodes_it)->pop(); 
    }
  }

  void LocalMap::draw(DrawAttributesType attributes, int name){
    if (! (attributes&ATTRIBUTE_SHOW))
      return;
    
    if (name>-1)
      glPushName(name);

    glPushMatrix();
    nicp::glMultMatrix(_transform);
    if (attributes&ATTRIBUTE_SELECTED) {
      if(!(attributes&ATTRIBUTE_ONLY)) {
	cloud()->draw(attributes);      
      }
      _nodes.draw(attributes);
      for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++) {
	(*it)->draw();
      }
    }

    glPushMatrix();
    glScalef(0.2, 0.2, 0.2);
    glPushAttrib(GL_COLOR);
    nicp::drawReferenceSystem();

    glPopAttrib();
    glPopMatrix();

    glPopMatrix();

    if (name>-1)
      glPopName();
  }

  BOSS_REGISTER_CLASS(LocalMap);  

}
