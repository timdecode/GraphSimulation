// Copyright 2016 Timothy Davison, all rights reserved.

#include "RegionGrowing.h"

#include "ShipEditorSimulation/MeshSimulation.h"
#include "GraphCopyContext.h"

void FGraphCopyContext::clear()
{
	duplicatedNodes.Empty();
}

FGraphCopyContext FGraphCopyContext::copySubgraph(
	FGraph& sourceGraph,
	FGraph& targetGraph,
	TArray<int32>& nodes,
	TArray<int32>& edges,
	const FTransform& localTransform,
	const FQuat& orientation,
	FMirrorAxis mirrorAxis)
{
	FGraphCopyContext context;

	std::unordered_map<int32, int32> toTargetNodes;
	std::unordered_map<int32, int32> toTargetEdges;

	float scale = FMath::Abs( localTransform.GetScale3D().GetMin() );

	targetGraph.beginTransaction();

	// copy the edges first (we don't care about edge.a or edge.b yet)
	for(int32 ei : edges)
	{
		// copy the edge, we don't want a reference inside of the array, because we could copy back into the array
		// and Unreal hates that (asserts on targetGraph.privateEdges.Add(edge) if targetGraph == sourceGraph. 
		FGraphEdge edge = sourceGraph.privateEdges[ei];

		if(!edge.isValid())
			continue;

		FGraphNode& nodeA = sourceGraph.allNodes[edge.a];
		FGraphNode& nodeB = sourceGraph.allNodes[edge.b];

		auto targetIndex = targetGraph.privateEdges.Add( edge );
		FGraphEdge& targetEdge = targetGraph.privateEdges[targetIndex];

		targetEdge.halfExtents *= scale;

		toTargetEdges[ei] = targetIndex;
	}

	// copy the nodes
	FTransform unflippedTransform = localTransform;
	unflippedTransform.SetScale3D( unflippedTransform.GetScale3D().GetAbs() );


	FVector centroid = FVector::ZeroVector;

	for(int32 ni : nodes)
	{
		// again, don't use references here, because we might end up copying back into the same array
		FGraphNode sourceNode = sourceGraph.allNodes[ni];

		if(!sourceNode.isValid())
			continue;

		int32 targetIndex = targetGraph.allNodes.Add( sourceNode );
		FGraphNode& targetNode = targetGraph.allNodes[targetIndex];

		targetNode.position = localTransform.TransformPosition( sourceNode.position );

		FQuat targetOrientation = orientation * sourceNode.orientation;
		targetNode.orientation = mirrorAxis.mirror( targetOrientation );

		targetNode.scale = scale * sourceNode.scale;
		targetNode.id = targetIndex;

		// this will be populated in the next step
		targetNode.edges.Empty();

		// copy the components
		// this will become a method on the components themselves
		targetNode.components.Empty();

		for(ComponentType type : sourceNode.components)
		{
			FGraphObject * component = sourceNode.component( sourceGraph, type );
			FGraphObject * targetComponent = targetNode.addComponent( targetGraph, type );

			UScriptStruct * scriptStruct = FGraphObject::componentStruct( type );

			// it's very important that the C++ constructor gets invoked
			scriptStruct->GetCppStructOps()->Construct( targetComponent );

			scriptStruct->CopyScriptStruct( targetComponent, component );

			targetComponent->nodeIndex = targetNode.id;
		}

		// update the centroid
		centroid += targetNode.position;

		// update the mapping
		toTargetNodes[sourceNode.id] = targetIndex;

		context.duplicatedNodes.Add( FGraphNodeHandle(targetIndex) );
	}

	centroid /= float( sourceGraph.allNodes.Num() );

	// link the edges and nodes up
	for(auto pair : toTargetEdges)
	{
		FGraphEdge& edge = sourceGraph.privateEdges[pair.first];
		FGraphEdge& targetEdge = targetGraph.privateEdges[pair.second];

		targetEdge.a = toTargetNodes[edge.a];
		targetEdge.b = toTargetNodes[edge.b];

		FGraphNode& targetA = targetGraph.allNodes[targetEdge.a];
		FGraphNode& targetB = targetGraph.allNodes[targetEdge.b];

		targetA.edges.Add( pair.second );
		targetB.edges.Add( pair.second );
	}

	targetGraph.endTransaction();

	return context;
}