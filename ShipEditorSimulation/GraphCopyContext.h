// Copyright 2016 Timothy Davison, all rights reserved.

#pragma once

#include "Graph.h"
#include "MirrorPoints.h"

/**
 * 
 */
struct REGIONGROWING_API FGraphCopyContext
{
public:
	void clear();

public:
	// The duplicated nodes in the target graph.
	TArray<FGraphNodeHandle> duplicatedNodes; 

public:
	static FGraphCopyContext copySubgraph(
		FGraph& sourceGraph,
		FGraph& targetGraph,
		TArray<int32>& nodes,
		TArray<int32>& edges,
		const FTransform& localTransform,
		const FQuat& orientation,
		FMirrorAxis mirrorAxis );
};
