// Copyright 2016 Timothy Davison, all rights reserved.

#pragma once

#include "Graph.h"

#include <unordered_map>
#include "ObjectIndexToInstancedStaticMeshInstance.h"

#include "MeshSimulation.generated.h"

class UInstancedStaticMeshComponent;

struct FGraphMeshKey
{
	UStaticMesh * staticMesh;
	UMaterialInterface * material;

	bool operator==( const FGraphMeshKey& other ) const
	{
		return staticMesh == other.staticMesh && material == other.material;
	}
};



USTRUCT( BlueprintType )
struct REGIONGROWING_API FGraphMesh : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	UStaticMesh * staticMesh = nullptr;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	UMaterialInterface * material = nullptr;

	operator FGraphMeshKey() { return FGraphMeshKey{ staticMesh, material }; }

	UInstancedStaticMeshComponent * _transientInstanceMesh = nullptr;

};



namespace std
{
	template<> struct hash<FGraphMeshKey>
	{
		std::size_t operator()( const FGraphMeshKey& key ) const
		{
			using std::hash;

			return hash<UStaticMesh*>()(key.staticMesh) ^ hash<UMaterialInterface*>()(key.material);
		}
	};
}

/**
 * 
 */
UCLASS()
class REGIONGROWING_API UMeshSimulation : public UObjectSimulation
{
	GENERATED_BODY()

public:
	// ---------------------------------------------
	// UMeshSimulation
	// ---------------------------------------------
	virtual void init();

	virtual void tick( float deltaT ) override;

	virtual bool canRunInEditor();

	void updateInstances();

	void _loadMeshes();

	void _addMesh( FGraphNode& node, FGraphMesh& mesh );

	// ---------------------------------------------
	// Component Listener Interface
	// ---------------------------------------------
	virtual void componentAdded( FGraphNodeHandle nodeHandle, ComponentType type );

	virtual void componentRemoved( FGraphNodeHandle node, ComponentType type );

	virtual void componentUpdated( FGraphNodeHandle node, ComponentType type );

	TArray<UInstancedStaticMeshComponent*> instancedStaticMeshes();

protected:
	UInstancedStaticMeshComponent * getOrCreateInstancedMesh( FGraphMesh& graphMesh );

	std::unordered_map<FGraphMeshKey, UInstancedStaticMeshComponent*> _instancedStaticeMeshes;
};
