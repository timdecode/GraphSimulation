// Copyright 2016 Timothy Davison, all rights reserved.

#include "RegionGrowing.h"

#include "MappedInstancedStaticMesh.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"

#include "MeshSimulation.h"

void UMeshSimulation::init()
{
	graph->addListener<FGraphMesh>( this );

	_loadMeshes();
}

void UMeshSimulation::tick( float deltaT )
{
	updateInstances();
}

bool UMeshSimulation::canRunInEditor()
{
	return true;
}

void UMeshSimulation::updateInstances()
{
	auto& meshes = graph->componentStorage<FGraphMesh>();

	USceneComponent * root = actor->GetRootComponent();
	FTransform rootTransform = root->GetComponentToWorld();

	std::unordered_map<UInstancedStaticMeshComponent*, int> _indexTable;

	for(FGraphMesh& mesh : meshes)
	{
		FGraphNode& node = graph->node( mesh.nodeIndex );

		FTransform transform( node.orientation, node.position, FVector( node.scale ) );

		UInstancedStaticMeshComponent * ismc = mesh._transientInstanceMesh;

		int& index = _indexTable[ismc];

		ismc->UpdateInstanceTransform( index, transform, false, false, true );

		index++;
	}

	for(auto& pair : _instancedStaticeMeshes)
		pair.second->MarkRenderStateDirty();
}

void UMeshSimulation::_loadMeshes()
{
	auto& meshes = graph->componentStorage<FGraphMesh>();

	for(FGraphMesh& mesh : meshes)
	{
		FGraphNode& node = graph->node( mesh.nodeIndex );

		_addMesh( node, mesh );
	}
}

void UMeshSimulation::_addMesh( FGraphNode& node, FGraphMesh& mesh )
{
	UInstancedStaticMeshComponent * instancedMesh = getOrCreateInstancedMesh( mesh );

	FTransform transform(node.orientation, node.position, FVector( node.scale ) );

	instancedMesh->AddInstance( transform );

	mesh._transientInstanceMesh = instancedMesh;
}

void UMeshSimulation::componentAdded( FGraphNodeHandle nodeHandle, ComponentType type )
{
	if(componentType<FGraphMesh>() != type)
		return;

	FGraphNode& node = nodeHandle( graph );

	FGraphMesh& mesh = node.component<FGraphMesh>( *graph );

	_addMesh( node, mesh );
}

void UMeshSimulation::componentRemoved( FGraphNodeHandle nodeHandle, ComponentType type )
{
	if(componentType<FGraphMesh>() != type)
		return;

	FGraphNode& node = nodeHandle( graph );

	FGraphMesh& meshComponent = node.component<FGraphMesh>( *graph );

	UInstancedStaticMeshComponent * mesh = meshComponent._transientInstanceMesh;

	if( mesh->GetInstanceCount() )
		mesh->RemoveInstance( mesh->GetInstanceCount() - 1 );
}

void UMeshSimulation::componentUpdated( FGraphNodeHandle nodeHandle, ComponentType type )
{

}

TArray<UInstancedStaticMeshComponent*> UMeshSimulation::instancedStaticMeshes()
{
	TArray<UInstancedStaticMeshComponent*> meshes;

	for(auto& pair : _instancedStaticeMeshes)
		meshes.Add( pair.second );

	return meshes;
}

UInstancedStaticMeshComponent * UMeshSimulation::getOrCreateInstancedMesh( FGraphMesh& graphMesh )
{
	UInstancedStaticMeshComponent * instancedMesh = nullptr;

	auto found = _instancedStaticeMeshes.find( graphMesh );

	if(found == _instancedStaticeMeshes.end())
	{
		instancedMesh = NewObject<UInstancedStaticMeshComponent>( actor );

		instancedMesh->SetStaticMesh( graphMesh.staticMesh );
		instancedMesh->SetMaterial( 0, graphMesh.material );
		instancedMesh->SetMobility( EComponentMobility::Movable );
		instancedMesh->SetCollisionEnabled( ECollisionEnabled::NoCollision );
		instancedMesh->UseDynamicInstanceBuffer = true;
		instancedMesh->KeepInstanceBufferCPUAccess = true;
		instancedMesh->SetCollisionProfileName( TEXT( "NoCollision" ) );
		instancedMesh->AttachToComponent( actor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform );
		instancedMesh->RegisterComponent();

		instancedMesh->SetRelativeScale3D( instancedMesh->GetRelativeTransform().GetScale3D() );

		_instancedStaticeMeshes[graphMesh] = instancedMesh;
	}
	else
		instancedMesh = found->second;

	return instancedMesh;
}
