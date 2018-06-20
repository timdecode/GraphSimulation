// Copyright 2016 Timothy Davison, all rights reserved.

#include "RegionGrowing.h"
#include "Graph.h"

void FGraph::init( UActorComponent * owner_in )
{
	owner = owner_in;

	_initDefaultTransaction();

	// allocate our component storage for all loaded component classes
	TArray<UScriptStruct*> derived;
	ShipEditorUtility::derivedStructs( FGraphObject::StaticStruct(), derived );

	for(UScriptStruct * typeStruct : derived)
		componentStorage( FGraphObject::componentType( typeStruct ) );

	// nuke classes that might not be there anymore after code changes
	simulations.Remove( nullptr );

	for(auto simulation : simulations)
		simulation->graph = this;

	for(auto simulation : simulations)
	{
		if(owner->GetWorld()->WorldType != EWorldType::Editor)
			simulation->init();
		else if(simulation->canRunInEditor())
			simulation->init();
	}
}

void FGraph::clear()
{
	beginTransaction();

	for(FGraphNode& node : allNodes)
	{
		if( node.isValid() )
			removeNode( node.id );
	}

	endTransaction();
}

void FGraph::instantiateSimulations()
{
	for(TObjectIterator<UClass> it; it; ++it)
	{
		if(!it->IsChildOf( UObjectSimulation::StaticClass() ) || *it == UObjectSimulation::StaticClass())
			continue;


		registerSimulation( *it, GetTransientPackage() );
	}
}

FComponentStorage& FGraph::componentStorage( ComponentType type )
{
	if(type >= _componentStorage.Num())
		_componentStorage.SetNum( type + 1 );

	FComponentStorage& storage = _componentStorage[type];

	if(storage.componentClass == nullptr)
	{
		UScriptStruct * typeStruct = FGraphObject::componentStruct( type );
		storage.componentClass = typeStruct;
	}

	return storage;
}

FGraphNodeHandle FGraph::anyNode()
{
	for(FGraphNode& node : allNodes)
	{
		if(!node.isValid())
			continue;

		return FGraphNodeHandle( node.id );
	}

	return FGraphNodeHandle();
}

int32 FGraph::addNode(FVector & position, FQuat orientation, float scale )
{
	int32 nodeIndex = RecyclingArray::emplace( allNodes, recycledNodes, position, orientation, scale );

	FGraphNode& node = allNodes[nodeIndex];
	node.id = nodeIndex;

    _transaction().addNode( nodeIndex );

	return nodeIndex;
}

void FGraph::removeNode( int32 index )
{
	if(!allNodes[index].isValid())
		return;

	FGraphNode nodeCopy = allNodes[index];

	allNodes[index]._isValid = false;

    _removeNode( index, _transaction().removalMacro );

    allNodes[index].removeComponents( *this );

	_transaction().removeNode( index );

	for(auto i : nodeCopy.edges)
		removeConnection( i );
}

void FGraph::markNodeDirty( int32 nodeIndex )
{
    if(GraphTransaction * transaction = _currentTransaction())
        transaction->dirtyNodes.push_back( nodeIndex );
    else
    {
        for(auto listener : _nodeListeners)
            listener->nodeUpdated( allNodes[nodeIndex] );
    }

	FGraphNode& graphNode = node( nodeIndex );
	for(ComponentType type : graphNode.components)
	{
		if(GraphTransaction * transaction = _currentTransaction())
		{
			transaction->updateComponents.emplace_back( nodeIndex, type );
		}
		else
		{
			for(auto listener : _componentListeners[type])
				listener->componentUpdated( FGraphNodeHandle( nodeIndex ), type );
		}
	}
}

int32 FGraph::connectNodes( int32 a, int32 b, int32 type, float halfExtents )
{
	int32 index = RecyclingArray::emplace( privateEdges, recycledEdges, a, b, type, halfExtents );

	FGraphNode& nodeA = allNodes[a];
	FGraphNode& nodeB = allNodes[b];

	nodeA.edges.Add( index );
	nodeB.edges.Add( index );

	GraphTransaction * transaction = _currentTransaction();

	if(transaction)
		transaction->addedConnections.push_back( index );
	else
		connectionAdded( index );

	return index;
}

void FGraph::removeConnection( int32 index )
{
    if(!privateEdges[index].isValid())
        return;

	FGraphEdge edgeCopy = privateEdges[index];

    connectionRemoved( index );

	if(!RecyclingArray::remove( privateEdges, recycledEdges, index ))
		return;

	FGraphNode& a = allNodes[edgeCopy.a];
	FGraphNode& b = allNodes[edgeCopy.b];

	a.edges.Remove( index );
	b.edges.Remove( index );

	if(a.edges.Num() == 0)
		removeNode( edgeCopy.a );
	if(b.edges.Num() == 0)
		removeNode( edgeCopy.b );
}

// -----------------------------------
// Graph Modification Macro Operations
// -----------------------------------

void FGraph::_removeNode( int32 index, RemovalMacro& macro )
{
    if(macro.removedNodes.find( index ) != macro.removedNodes.end())
        return;

    FGraphNode& node = allNodes[index];
    macro.removedNodes.insert( index );

    for(auto i : node.edges)
        _removeConnection( i, macro );
}

void FGraph::_removeConnection( int32 index, RemovalMacro& macro )
{
    if(macro.removedConnections.find( index ) != macro.removedConnections.end())
        return;

    FGraphEdge& edge = privateEdges[index];

    macro.removedConnections.insert( index );
    
    if(macro.modifiedNodes.find( edge.a ) == macro.modifiedNodes.end())
        macro.modifiedNodes[edge.a] = allNodes[edge.a];

    if(macro.modifiedNodes.find( edge.b ) == macro.modifiedNodes.end())
        macro.modifiedNodes[edge.b] = allNodes[edge.b];

    // work on copies of the original
    FGraphNode& a = macro.modifiedNodes[edge.a];
    FGraphNode& b = macro.modifiedNodes[edge.b];

    a.edges.Remove( index );
    b.edges.Remove( index );

    if(a.edges.Num() == 0)
        _removeNode( edge.a, macro );
    if(b.edges.Num() == 0)
        _removeNode( edge.b, macro );
}

void FGraph::_dispatch( RemovalMacro& macro )
{

}

void FGraph::_commit( RemovalMacro& macro )
{

}



int32 FGraph::edgeBetween( int32 a, int32 b )
{
	FGraphNode& nodeA = allNodes[a];
	FGraphNode& nodeB = allNodes[b];

	if(!nodeA.isValid() || !nodeB.isValid())
		return -1;

	for(auto ei : nodeA.edges)
	{
		FGraphEdge& edge = privateEdges[ei];

		if(!edge.isValid())
			continue;

		if((edge.a == a && edge.b == b) || (edge.a == b && edge.b == a))
			return ei;
	}

	return -1;
}

FGraph& FGraph::operator=( const FGraph& other )
{
	allNodes = other.allNodes;
	recycledNodes = other.recycledNodes;
	privateEdges = other.privateEdges;
	recycledEdges = other.recycledEdges;

	_componentStorage = other._componentStorage;

	simulations.Empty();
	for(UObjectSimulation * simulation : other.simulations)
	{
		UObjectSimulation * duplicateSimulation = DuplicateObject( simulation, nullptr );
		simulations.Add( duplicateSimulation );
	}

	return *this;
}

// ------------------------------------------------------------
// FGraphObject
// ------------------------------------------------------------
void FGraphObject::removedFromNode( FGraphNode & node, FGraph & graph )
{

}

UScriptStruct* FGraphObject::componentStruct( ComponentType type )
{
	return StructTypeManager<FGraphObject>::structForType( type );
}

ComponentType FGraphObject::componentType( UScriptStruct* typeStruct )
{
	return StructTypeManager<FGraphObject>::typeForStruct( typeStruct );
}

FGraphNodeHandle::FGraphNodeHandle( FGraphNode& node )
{
	index = node.id;
}

FGraphNode& FGraphNodeHandle::operator()( FGraph& graph )
{
	return graph.allNodes[index];
}

FGraphNode& FGraphNodeHandle::operator()( FGraph* graph )
{
	return graph->allNodes[index];
}


FGraphNodeHandle FGraphEdge::other( FGraphNodeHandle node )
{
	bool isA = node.index == a;
	bool isB = node.index == b;

	assert( isA || isB );

	return FGraphNodeHandle( isA ? b : a );
}
