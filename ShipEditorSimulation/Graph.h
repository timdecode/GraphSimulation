// Copyright 2016 Timothy Davison, all rights reserved.

#pragma once

#include <vector>
#include <stdint.h>
#include <limits>
#include <memory>

#include <unordered_map>
#include <unordered_set>

#include "ObjectRecord.h"
#include "TypeId.h"
#include "Pool.h"
#include "Utility.h"
#include "HalfEdgeMesh.h"
#include "GraphVersion.h"

#include "Graph.generated.h"

class UGraphComponent;
class UObjectSimulation;
struct FGraph;

struct FGraphObject;
struct FGraphNode;


// A convenience class for keeping simulation state objects associated with FGraphObject indices.
//
// The order of the elements in the array will change as elements are added or removed, however
// a secondary map is used to uniquely map each element to an unique index on emplace.
// The elements must have an nodeIndex property for this to work.
// This enables very fast iteration of elements for simulation, while also maintaining fast
// insert/delete and a mapping to FGraphObject indices.
template<typename ElementType>
struct MappedArray
{
private:
	TArray<ElementType> * _elements;
	std::unordered_map<int32, int32> _objectIndexToElement;

public:
	void init( TArray<ElementType>& elements )
	{
		_elements = &elements; 

		int32 n = elements.Num();

		_objectIndexToElement.clear();
		for(int32 elementIndex = 0; elementIndex < n; ++elementIndex)
		{
			ElementType& element = elements[elementIndex];
			_objectIndexToElement.emplace( element.nodeIndex, elementIndex );
		}
	}

	template<class... Args>
	ElementType& emplace( int32 nodeIndex, Args&&... args )
	{
		int32 elementIndex = _elements->Emplace( std::forward<Args>( args )... );
		ElementType& element = (*_elements)[elementIndex];
		element.nodeIndex = nodeIndex;

		_objectIndexToElement.emplace( nodeIndex, elementIndex );

		return (*_elements)[elementIndex];
	}

	void erase( int32 nodeIndex )
	{
		auto found = _objectIndexToElement.find( nodeIndex );
		if(found == _objectIndexToElement.end())
			return;

		int32 n = _elements->Num();

		if(n == 1)
		{
			_elements->RemoveAt( 0 );
			_objectIndexToElement.erase( found );
		}
		else
		{
			int32 elementIndex = found->second;

			// last element, we can just kill it
			if(elementIndex == n - 1)
				_elements->RemoveAt( elementIndex );
			else
			{
				// middle element, so we efficiently swap in the last element of the array
				// and remove the element while update the _objectIndexToElement map with
				// the swapped element index
				_elements->RemoveAtSwap( elementIndex );

				ElementType& element = (*_elements)[elementIndex];

				_objectIndexToElement[element.nodeIndex] = elementIndex;
			}

			_objectIndexToElement.erase( found );
		}
	}

	int32 elementIndex( int32 nodeIndex )
	{
		return _objectIndexToElement[nodeIndex];
	}

	ElementType& at( int32 nodeIndex )
	{
		return (*_elements)[_objectIndexToElement[nodeIndex]];
	}

	bool contains( int32 nodeIndex )
	{
		return _objectIndexToElement.find( nodeIndex ) != _objectIndexToElement.end();
	}

	void clear()
	{
		_objectIndexToElement.clear();
		_elements->Empty();
	}
};

typedef StructTypeManager<FGraphObject>::TypeId ComponentType;

USTRUCT( BlueprintType )
struct REGIONGROWING_API FGraphObject 
{
	GENERATED_BODY()

	virtual ~FGraphObject() {}

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	int32 nodeIndex = 0;

    // called after a graph object is removed from a node and before the destructor
    virtual void removedFromNode( struct FGraphNode& node, FGraph& graph );

	static UScriptStruct* componentStruct( ComponentType type );

	static ComponentType componentType( UScriptStruct* typeStruct );
};


template<class TComponent>
inline static ComponentType componentType()
{
	static_assert(std::is_base_of<FGraphObject, TComponent>::value, "TComponent must be derived from TGraphComponent.");

	return StructTypeManager<FGraphObject>::typeId<TComponent>(); 
}

template<class TComponent>
inline static UScriptStruct* componentStruct()
{
	static_assert(std::is_base_of<FGraphObject, TComponent>::value, "TComponent must be derived from TGraphComponent.");

	ComponentType type = componentType<TComponent>();
	return StructTypeManager<FGraphObject>::structForType( type );
}

USTRUCT( BlueprintType )
struct REGIONGROWING_API FGraphNodeHandle
{
	GENERATED_USTRUCT_BODY()

	FGraphNodeHandle() {}
	explicit FGraphNodeHandle( int32 i ) : index( i ) {}

	explicit FGraphNodeHandle( FGraphNode& node );

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	int32 index = -1;

	inline explicit operator bool() const
	{
		return index >= 0;
	}

	FGraphNode& operator() ( FGraph& mesh );
	FGraphNode& operator() ( FGraph* mesh );

	friend bool operator<( const FGraphNodeHandle& a, const FGraphNodeHandle& b ) 
	{
		return a.index < b.index;
	}

};

namespace std
{
	template<> struct hash<FGraphNodeHandle>
	{
		std::size_t operator()( const FGraphNodeHandle& handle ) const
		{
			return std::hash<decltype(handle.index)>()(handle.index);
		}
	};
}

USTRUCT( BlueprintType )
struct REGIONGROWING_API FGraphNode
{
	GENERATED_USTRUCT_BODY()

public:
	FGraphNode() : position( FVector::ZeroVector )
	{
		edges.Empty();
	};

	FGraphNode( FVector& position_in ) : position( position_in )
	{
		edges.Empty();
	}

	FGraphNode( FVector& position_in, FQuat& orientation_in, float scale_in ) : position( position_in ), orientation( orientation_in ), scale( scale_in )
	{
		edges.Empty();
	}

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	int32 id = 0;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Meta = (MakeEditWidget = true), Category = "ShipEditor" )
	FVector position = FVector::ZeroVector;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	FQuat orientation = FQuat::Identity;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	float scale = 1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TArray<int32> edges;

	template<class TComponent>
	TComponent& component( FGraph& graph );

	FGraphObject* component( FGraph& graph, ComponentType type );

	template<class TComponent>
	bool hasComponent();

	bool hasComponent( ComponentType type );

	FGraphObject* addComponent( FGraph& graph, ComponentType type );

	template<class TComponent, class... TArgs>
	TComponent& addComponent( FGraph& graph, TArgs... args );

	template<class TComponent>
	void removeComponent( FGraph& graph );

	void removeComponent( FGraph& graph, ComponentType type );


	void removeComponents( FGraph& graph )
	{
        TArray<int32> copy(components);

		for(ComponentType type : copy)
			removeComponent( graph, type );
	}

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	bool _isValid = true;

	bool isValid() const { return _isValid; };
	void invalidate() { _isValid = false; edges.Empty(); };

	// Component types attached to this node, this will be populated during post serialize.
	// For simplicity of dealing with added/removed component classes, this is transient.
	UPROPERTY( EditAnywhere, BlueprintReadOnly, Transient, Category = "ShipEditor" )
	TArray<int32> components; 


	operator FGraphNodeHandle() { return FGraphNodeHandle( id ); }
};


typedef int32 NodeIndex;

USTRUCT( BlueprintType )
struct FComponentStorage
{
	GENERATED_USTRUCT_BODY()

protected:


	std::unordered_map<int32, int32> _objectIndexToElement;

public:
	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	FPoolBase pool;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	int32 _size = 0;


	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UScriptStruct * componentClass = nullptr;

	bool Serialize( FArchive& Ar )
	{
		Ar.UsingCustomVersion( FGraphVersion::GUID );

		Ar << componentClass;
		Ar << _size;

		if(Ar.CustomVer( FGraphVersion::GUID ) > FGraphVersion::Initial)
		{
			int32 structSize = componentClass ? componentClass->GetStructureSize() : 0;

			Ar << structSize;

			if(componentClass == nullptr && Ar.IsLoading() )
			{
				int64 position = Ar.Tell();

				position += structSize * _size;

				Ar.Seek( position );

				_size = 0;
				return true;
			}
		}
		else
		{
			// was the class removed?
			if(componentClass == nullptr)
			{
				_size = 0;

				return true;
			}
		}

		pool.resize( _size, componentClass->GetStructureSize() );

		for(size_t i = 0; i < _size; ++i)
		{
			FGraphObject * object = at( i, componentClass );

			// very important to call the C++ constructor
			if(Ar.IsLoading())
				componentClass->GetCppStructOps()->Construct( object );

			componentClass->SerializeItem( Ar, object, nullptr );

			_objectIndexToElement.emplace( object->nodeIndex, i );
		}

		return true;
	}

	void PostSerialize( const FArchive& Ar )
	{
		_objectIndexToElement.clear();

		for(size_t i = 0; i < _size; ++i)
		{
			FGraphObject * object = at( i, componentClass );

			_objectIndexToElement.emplace( object->nodeIndex, i );
		}
	}

	FComponentStorage& operator=( const FComponentStorage& other )
	{
		_size = other._size;
		componentClass = other.componentClass;

		pool = other.pool;

		_objectIndexToElement.clear();

		for(size_t i = 0; i < _size; ++i)
		{
			FGraphObject * object = at( i, componentClass );

			_objectIndexToElement.emplace( object->nodeIndex, i );
		}

		return *this;
	}

	bool Identical( const FComponentStorage* other, uint32 PortFlags ) const
	{
		if(!other)
			return false;

		if(componentClass != other->componentClass)
			return false;

		if(!componentClass)
			return true;

		if(_size != other->_size)
			return false;

		return pool == other->pool;
	}

	void AddStructReferencedObjects( class FReferenceCollector& collector ) const
	{
		auto unconstThis = const_cast<FComponentStorage*>(this);

		collector.AddReferencedObject( unconstThis->componentClass );

		if(componentClass)
		{
			for(size_t i = 0; i < _size; ++i)
			{
				FGraphObject * object = unconstThis->at( i, componentClass );

				componentClass->SerializeBin( collector.GetVerySlowReferenceCollectorArchive(), object );
			}


			//FSimpleObjectReferenceCollectorArchive objectReferenceCollector( nullptr, collector );

			//for(size_t i = 0; i < _size; ++i)
			//{
			//	FGraphObject * object = unconstThis->at( i, componentClass );

			//	componentClass->SerializeBin( objectReferenceCollector, object );
			//}
		}
	}

	size_t size()
	{
		return _size;
	}

	FGraphObject* emplace( FGraphNode& node, UScriptStruct * typeStruct )
	{
		size_t componentSize = typeStruct->GetStructureSize();
		auto nodeIndex = node.id;

		pool.resize( _size + 1, componentSize );

		FGraphObject * component = static_cast<FGraphObject*>(pool.get( _size, componentSize ));

		typeStruct->InitializeStruct( (uint8*)component ); 

		int32 componentIndex = _size;

		_size++;

		component->nodeIndex = nodeIndex;

		_objectIndexToElement.emplace( nodeIndex, componentIndex );

		return component;
	}


	template<class TComponent, class... Args>
	TComponent& emplace( FGraphNode& node, Args&&... args )
	{
		static_assert(std::is_base_of<FGraphObject, TComponent>::value, "TComponent must derive from FGraphObject.");

		auto nodeIndex = node.id;

		Pool<TComponent> * typedPool = static_cast< Pool<TComponent>* >(&pool);

		typedPool->resize( _size + 1 );

		TComponent * component = new(typedPool->get( _size )) TComponent( std::forward<Args>( args )... );

		int32 componentIndex = _size;

		_size++;

		component->nodeIndex = nodeIndex;

		_objectIndexToElement.emplace( nodeIndex, componentIndex );

		return *component;
	}

	void erase( FGraphNode& node, ComponentType type )
	{
		auto nodeIndex = node.id;

		auto found = _objectIndexToElement.find( nodeIndex );
		if(found == _objectIndexToElement.end())
			return;

		UScriptStruct * scriptStruct = FGraphObject::componentStruct( type );

		size_t componentSize = scriptStruct->GetStructureSize();

		if(_size == 1)
		{
			scriptStruct->DestroyStruct( pool.get( 0, componentSize ) );
			
			_size = 0;

			_objectIndexToElement.erase( found );
		}
		else
		{
			int32 componentIndex = found->second;

			// last element, we can just kill it
			if(componentIndex == _size - 1)
			{
				scriptStruct->DestroyStruct( pool.get( componentIndex, componentSize ) );
				_size--;
			}
			else
			{
				scriptStruct->DestroyStruct( pool.get( componentIndex, componentSize ) );

				// copy the last element to this spot in the pool
				FMemory::Memcpy(
					pool.get( componentIndex, componentSize ),
					pool.get( _size - 1, componentSize ),
					componentSize
				);

				_size--;

				FGraphObject * component = static_cast<FGraphObject*>(pool.get( componentIndex, componentSize ));

				_objectIndexToElement[component->nodeIndex] = componentIndex;
			}

			_objectIndexToElement.erase( found );
		}

		pool.resize( _size, componentSize );
	}

	template<class TComponent>
	void erase( FGraphNode& node )
	{
		static_assert(std::is_base_of<FGraphObject, TComponent>::value, "TComponent must derive from FGraphObject.");

		se::Pool<TComponent> * typedPool = static_cast< se::Pool<TComponent>* >(&pool);

		auto nodeIndex = node.id;

		auto found = _objectIndexToElement.find( nodeIndex );
		if(found == _objectIndexToElement.end())
			return;

		if(_size == 1)
		{
			typedPool->destroy( 0 );
			_size = 0;

			_objectIndexToElement.erase( found );
		}
		else
		{
			int32 componentIndex = found->second;

			// last element, we can just kill it
			if(componentIndex == _size - 1)
			{
				typedPool->destroy( componentIndex );
				_size--;
			}
			else
			{
				typedPool->destroy( componentIndex );

				// copy the last element to this spot in the pool
				FMemory::Memcpy(
					typedPool->get( componentIndex ),
					typedPool->get( _size - 1),
					sizeof( TComponent )
				);

				_size--;

				TComponent * component = typedPool->get( componentIndex );

				_objectIndexToElement[component->nodeIndex] = componentIndex;
			}

			_objectIndexToElement.erase( found );
		}

		typedPool->resize( _size );
	}

	int32 elementIndex( int32 nodeIndex )
	{
		return _objectIndexToElement[nodeIndex];
	}



	template<class TComponent>
	TComponent& at( int32 componentIndex )
	{
		static_assert(std::is_base_of<FGraphObject, TComponent>::value, "TComponent must derive from FGraphObject.");
		Pool<TComponent> * typedPool = static_cast< Pool<TComponent>* >(&pool);

		TComponent * component = typedPool->get( componentIndex );
		return *component;
	}

	FGraphObject* at( int32 componentIndex, UScriptStruct * componentStruct )
	{
		size_t size = componentStruct->GetStructureSize();

		return static_cast<FGraphObject*>(pool.get( componentIndex, size ));
	}

	FGraphObject* componentForNode( FGraphNode& node, ComponentType type )
	{
		auto found = _objectIndexToElement.find( node.id );
		
		if(found == _objectIndexToElement.end())
			return nullptr;
		 
		UScriptStruct * typeStruct = FGraphObject::componentStruct( type );

		return static_cast<FGraphObject*>(pool.get( found->second, typeStruct->GetStructureSize() ));
	}

	bool contains( int32 nodeIndex )
	{
		return _objectIndexToElement.find( nodeIndex ) != _objectIndexToElement.end();
	}

	template<class TComponent>
	void clear()
	{
		static_assert(std::is_base_of<FGraphObject, TComponent>::value, "TComponent must derive from FGraphObject.");
		Pool<TComponent>& typedPool = static_cast<Pool<TComponent>>(pool);

		_objectIndexToElement.clear();

		for(size_t i = 0; i < _size; ++i)
		{
			typedPool.destroy( i );
		}

		_size = 0;
		typedPool.resize( 0, 0 );
	}
};

template<>
struct TStructOpsTypeTraits<FComponentStorage> : public TStructOpsTypeTraitsBase2<FComponentStorage>
{
	enum
	{
		WithSerializer = true,
		WithCopy = true,
		WithIdentical = true,
		//WithAddStructReferencedObjects = true,
		WithAddStructReferencedObjects = true,
		WithPostSerialize = true
	};
};

template<class TComponent>
struct TypedComponentStorage : FComponentStorage
{
	TComponent* begin()
	{
		Pool<TComponent> * typedPool = static_cast<Pool<TComponent>*>(&pool);

		return typedPool->begin();
	}

	TComponent* end()
	{
		Pool<TComponent> * typedPool = static_cast<Pool<TComponent>*>(&pool);

		return typedPool->end();
	}

	TComponent& componentForNode( FGraphNode& node )
	{
		ComponentType type = componentType<TComponent>();

		return static_cast<TComponent&>(*FComponentStorage::componentForNode( node, type ));
	}

	TComponent& at( int32 i )
	{
		return FComponentStorage::at<TComponent>( i );
	}
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FGraphEdge
{
	GENERATED_USTRUCT_BODY()

public:
    FGraphEdge() {};
	FGraphEdge( int32 aIn, int32 bIn ) : a( aIn ), b( bIn ) {};
    FGraphEdge( int32 aIn, int32 bIn, int32 type_in, float halfExtents_in ) : a( aIn ), b( bIn ), type(type_in), halfExtents(halfExtents_in) {}; 

	bool isValid() const { return a >= 0; };
	void invalidate() { a = -1; };

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	int32 a = 0;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	int32 b = 0;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	int32 type = 0;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	float halfExtents = 0.0f;

	// given a node in this edge, returns the other node
	// will assert if the node is not in this edge
	FGraphNodeHandle other( FGraphNodeHandle node );
};

struct ComponentListener
{
	virtual void componentAdded( FGraphNodeHandle node, ComponentType type ) {};
	virtual void componentRemoved( FGraphNodeHandle node, ComponentType type ) {};
	virtual void componentUpdated( FGraphNodeHandle node, ComponentType type ) {};

	virtual void connectionAdded( int32 edgeIndex, FGraphEdge& edge, ComponentType type ) {};
	virtual void connectionRemoved( int32 edgeIndex, FGraphEdge& oldEdge, ComponentType type ) {};
};

struct EdgeListener
{
    virtual void connectionAdded( int32 edgeIndex, FGraphEdge& edge ) {};
    virtual void connectionUpdated( int32 edgeIndex, FGraphEdge& edge ) {};
    virtual void connectionRemoved( int32 edgeIndex, FGraphEdge& oldEdge ) {};
};

struct NodeListener
{
    virtual void nodeAdded( FGraphNode& node ) {};
    virtual void nodeUpdated( FGraphNode& node ) {};
    virtual void nodeRemoved( FGraphNode& oldNode ) {};
};



template<typename iterator_type, typename value_type>
class IsValidArray_iterator
{
	iterator_type _current;

public:
	IsValidArray_iterator( const iterator_type& begin) : _current( begin )
	{
		// move to the first valid item
		while((_current) && !(*_current).isValid())
			++_current;
	}

	const value_type& operator*() const
	{
		return *_current;
	}

	const value_type* operator->() const
	{
		return _current.operator->();
	}

	IsValidArray_iterator& operator++()
	{
		do
		{
			++_current;
		} while((_current) && !(*_current).isValid());

		return *this;
	}

	bool operator==( const IsValidArray_iterator& rhs ) const
	{
		return _current == rhs._current;
	}

	bool operator!=( const IsValidArray_iterator& rhs ) const
	{
		return !(operator==( rhs ));
	}

	/** conversion to "bool" returning true if the iterator has not reached the last element. */
	FORCEINLINE explicit operator bool() const
	{
		return (bool)_current;
	}
	/** inverse of the "bool" operator */
	FORCEINLINE bool operator !() const
	{
		return !(bool)*this;
	}

	int32 GetIndex() const
	{
		return _current.GetIndex();
	}
};

// A static struct with static methods to emplace/remove elements from an array without resizing the array.
// The indices of recycled elements are kept in an array.
//
// Unreal's property system does not support templated structs. So, this is a compromise on that where one
// passes the storage array, and the recycled index array to emplace and remove elements.
struct RecyclingArray
{
	template<typename ElementType, class... Args>
	static int32 emplace( TArray<ElementType>& elements, TArray<int32>& recycledIndices, Args&&... args )
	{
		int32 index = -1;

		int32 n = recycledIndices.Num();
		if(n)
		{
			index = recycledIndices[n - 1];
			recycledIndices.RemoveAt( n - 1 );
			new(elements.GetData() + index) ElementType( std::forward<Args>(args)... );
		}
		else
		{
			index = elements.Num();
			elements.Emplace( std::forward<Args>( args )... );
		}
			
		return index;
	}

	// returns true if we invalidated the object at index, returns false if the object was already invalidated 
	template<typename ElementType>
	static bool remove( TArray<ElementType>& elements, TArray<int32>& recycledIndices, int32 index )
	{
		ElementType& e = elements[index];

		if(!e.isValid())
			return false;

		e.invalidate();

		recycledIndices.Add( index );

		return true;
	}
};

/**
*
*/
UCLASS( BlueprintType, EditInlineNew, abstract )
class REGIONGROWING_API UObjectSimulation : public UObject, public ComponentListener
{
	GENERATED_BODY()

public:
	FGraph * graph = nullptr;
	AActor * actor = nullptr;

public:
	virtual bool canRunInEditor()
	{
		return false;
	}
	
	// these should be abstract methods, but unreal doesn't allow that
	virtual void init()
	{
	}



	virtual void tick( float deltaT )
	{

	}

	virtual void componentAdded( FGraphNodeHandle node, ComponentType type )
	{

	}

	virtual void componentRemoved( FGraphNodeHandle node, ComponentType type )
	{

	}

	virtual void componentUpdated( FGraphNodeHandle node, ComponentType type )
	{

	}
};


/**
 * 
 */
USTRUCT( BlueprintType )
struct REGIONGROWING_API FGraph
{
	GENERATED_BODY()

public:
	typedef IsValidArray_iterator<TArray<FGraphNode>::TConstIterator, FGraphNode> nodeIterator;
	typedef IsValidArray_iterator<TArray<FGraphEdge>::TConstIterator, FGraphEdge> edgeIterator;

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TArray<FGraphNode> allNodes;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TArray<int32> recycledNodes;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TArray<FGraphEdge> privateEdges;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TArray<int32> recycledEdges;

	UPROPERTY( EditAnywhere, Instanced, BlueprintReadWrite, Category = "ShipEditor" )
	TArray<class UObjectSimulation*> simulations;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TArray<FComponentStorage> _componentStorage;

	// owner is set on init
	UActorComponent * owner = nullptr;

private:

	std::unordered_map<ComponentType, std::vector<ComponentListener*>> _componentListeners;
    std::vector<NodeListener*> _nodeListeners;
    std::vector<EdgeListener*> _edgeListeners;

public:
	void init( UActorComponent * owner_in);

	void clear();

	void instantiateSimulations();

	void resetListeners()
	{
		_componentListeners.clear();
		_nodeListeners.clear();
		_edgeListeners.clear();
	}

	FComponentStorage& componentStorage( ComponentType type );

	template<class TComponent>
	TypedComponentStorage<TComponent>& componentStorage()
	{
		ComponentType type = componentType<TComponent>();

		return static_cast<TypedComponentStorage<TComponent>&>(componentStorage( type ));
	}

	FGraphNodeHandle anyNode();

	// the returned index is only valid until the next change to the graph
	int32 addNode( FVector& position, FQuat orientation = FQuat::Identity, float scale = 1.0f);
	void removeNode( int32 index );
	FGraphNode& node( int32 index ) { return allNodes[index]; };
    void markNodeDirty( int32 index );

	int32 connectNodes( int32 a, int32 b, int32 type = 0, float halfExtents = 0.0f );
	void removeConnection( int32 index );
	FGraphEdge& edge( int32 index ) { return privateEdges[index]; };
	int32 edgeBetween( int32 nodeA, int32 nodeB );

	// returns all of the valid nodes
	nodeIterator nodes() 
	{
		return nodeIterator( allNodes.CreateConstIterator() );
	}

	int32 numNodes()
	{
		return allNodes.Num() - recycledNodes.Num();
	}

	edgeIterator edges()
	{
		return edgeIterator( privateEdges.CreateConstIterator() ); 
	}

	int32 numEdges()
	{
		return privateEdges.Num() - recycledEdges.Num();
	}

public:

    // ---------------------------------------------------------------------------
    // Graph Modification Macro Operations
    // -----------------------------------
    // Removing a node, might remove connections, so we do a first pass
    // where we simulate all of these modifications, store them in a macro, and
    // then apply a macro to a transaction
    // --------------------------------------------------------------------------- 
private:
    struct RemovalMacro
    {
        std::unordered_set<int32> removedNodes;
        std::unordered_set<int32> removedConnections;

        std::unordered_map<int32, FGraphNode> modifiedNodes;

        void clear()
        {
            removedNodes.clear();
            removedConnections.clear();
            modifiedNodes.clear();
        }
    };

    void _removeNode( int32 index, RemovalMacro& macro );
    void _removeConnection( int32 index, RemovalMacro& macro );

    void _dispatch( RemovalMacro& macro );
    void _commit( RemovalMacro& macro );

    struct Transaction
    {
        Transaction( FGraph * graph_in ) : graph( graph_in ) {}

    public:
        std::vector<int32> removedNodes;
        std::vector<int32> addedNodes;
        std::vector<int32> updatedNodes;

        std::vector<int32> removedConnections;
        std::vector<int32> addedConnections;
        std::vector<int32> updatedConnections;

    public:
        bool commitImmediately = false;
        FGraph * graph;
        RemovalMacro removalMacro;

        void addNode( int32 node )
        {
            addedNodes.push_back( node );

            if(commitImmediately)
            {
                graph->_commit( *this );

                addedNodes.clear();
            }
        }

        void removeNode( int32 node )
        {
            if(commitImmediately)
            {
                removedNodes.push_back( node );

                graph->_commit( *this );

                removedNodes.clear();
                removalMacro.clear();

				RecyclingArray::remove( graph->allNodes, graph->recycledNodes, node );
            }
            else if( removalMacro.removedNodes.find(node) == removalMacro.removedNodes.end() )
                removedNodes.push_back( node );
        }
    };

    struct TransactionListener
    {
        virtual void graphDidChange( Transaction& changeTransaction ) {};
    };

    void addListener( TransactionListener * listener )
    {
        if(std::find( std::begin( _transactionListeners ), std::end( _transactionListeners ), listener ) != std::end(_transactionListeners) )
            return;

        _transactionListeners.push_back( listener );
    }

    void removeListener( TransactionListener * listener )
    {
        _transactionListeners.erase( std::remove_if( 
            _transactionListeners.begin(), 
            _transactionListeners.end(),
            [=]( TransactionListener * l ) { return l == listener; } ) );
    }

    void clearListeners()
    {
        _transactionListeners.clear();
    }

private:
    std::vector<Transaction> _transactions;
    std::vector<TransactionListener*> _transactionListeners;

    void _initDefaultTransaction()
    {
        _transactions.clear();
        _transactions.emplace_back(this);

        Transaction& defaultTransaction = _transactions.back();
        defaultTransaction.commitImmediately;
    }

    Transaction& _transaction()
    {
		if(!_transactions.size())
			_initDefaultTransaction();

        return _transactions.back();
    }

    void _commit( Transaction& transaction )
    {
        for(auto listener : _transactionListeners)
            listener->graphDidChange( transaction );
    }

    void _removeGarbage( Transaction& transaction )
    {

    }

    void _processTransaction( Transaction& transaction )
    {
        _commit( transaction );
        _removeGarbage( transaction );
    }

public:
    /**
    Called like this:
    \code{.c++}
        graph.performTransaction([]{
            auto ni = graph.addNode(position, orientation, scale);
            FGraphNode& node = graph.node(ni);

            node.addComponent<FBobComponent>();
        });
    \endcode

    This will defer all operations inside of the passed lambda, until the lamda returns.

    Transactions are useful for batching graph operations into a single listener call, which
    can save overhead in those listeners.
    */
    template<typename TLambda>
    void performTransaction(TLambda&& transactionBlock)
    {
        _transactions.emplace_back(*this);

        transactionBlock();

        Transaction& transaction = _transaction();
        _processTransaction( transaction );

        _transactions.pop_back();
    }















































private:

	// ---------------------------------------------------------------------------
	// Transaction API
	// ---------------------------------------------------------------------------
private:

    struct GraphTransaction
	{
	public:
		std::vector<std::pair<NodeIndex, ComponentType>> addedComponents;
		std::vector<std::pair<NodeIndex, ComponentType>> removedComponents;
		std::vector<std::pair<NodeIndex, ComponentType>> updateComponents;

		std::vector<int32> addedConnections;




        //std::vector<EdgeTransaction> removedTypedConnections;
		std::vector<std::pair<int32, FGraphEdge>> removedConnections; // old index and a copy of the old edge

        std::vector<int32> addedNodes;
        std::vector<FGraphNode> removedNodes;   // this has to be a copy, since the node may have already been overwritten
        std::vector<int32> dirtyNodes;
	};

	std::vector<GraphTransaction> _graphTransactions;
	GraphTransaction * _currentTransaction()
	{
		size_t n = _graphTransactions.size();
		if(n)
			return &_graphTransactions[n - 1];
		else
			return nullptr;
	}

public:
	void beginTransaction()
	{
        _graphTransactions.emplace_back();
	}

	void endTransaction()
	{
		if(_graphTransactions.size() == 0)
			return;

		GraphTransaction& back = _graphTransactions.back();

		// dispatch added components
		for(auto& pair : back.addedComponents)
		{
			auto found = _componentListeners.find( pair.second );

			if(found == _componentListeners.end())
				continue;

			for(auto listener : found->second)
				listener->componentAdded( allNodes[pair.first], pair.second );
		}

		// dispatch removed components
		for(auto& pair : back.removedComponents)
		{
			auto found = _componentListeners.find( pair.second );

			if(found == _componentListeners.end())
				continue;

			for(auto listener : found->second)
				listener->componentRemoved( allNodes[pair.first], pair.second );
		}
		
		// dispatch added connections
		for(auto edgeIndex : back.addedConnections)
			connectionAdded( edgeIndex );

		// dispatch removed connections
		for(auto& pair : back.removedConnections)
			connectionRemoved( pair.first );

        // dispatch added nodes
        for(auto nodeIndex : back.addedNodes)
        {
            for(auto listener : _nodeListeners)
                listener->nodeAdded( allNodes[nodeIndex] );
        }

        // dispatch removed nodes
        for(auto& node : back.removedNodes)
        {
            for(auto listener : _nodeListeners)
                listener->nodeRemoved( node );
        }

        // dispatch dirty nodes
        for(auto nodeIndex : back.dirtyNodes)
        {
            for(auto listener : _nodeListeners)
                listener->nodeUpdated( allNodes[nodeIndex] );
        }

		// kill the removed nodes and components finally
		for(auto& pair : back.removedComponents)
		{
			componentStorage( pair.second ).erase( allNodes[pair.first], pair.second );
		}

		for(auto& node : back.removedNodes)
		{
			RecyclingArray::remove( allNodes, recycledNodes, node.id );
		}

        _graphTransactions.pop_back();
	}

	void didUpdateNode( NodeIndex index )
	{
		//GraphTransaction& back = _graphTransactions.back();
		//
		//back.dirtyNodes.push_back( index );

		UE_LOG( LogTemp, Warning, TEXT( "didUpdateNode not implemented in FGraph" ) );
	}

public:
	// ---------------------------------------------------------------------------
	// Component Listeners
	// ---------------------------------------------------------------------------
	template<class TComponent>
	void addListener( ComponentListener * listener )
	{
		addListener( listener, componentType<TComponent>() );
	}

	void addListener( ComponentListener * listener, ComponentType type )
	{
		auto found = _componentListeners.find( type );

		if(found == _componentListeners.end())
		{
			std::vector<ComponentListener*> listeners;
			listeners.emplace_back( listener );

			_componentListeners.emplace( type, listeners );
		}
		else
			found->second.emplace_back( listener );
	}

	void componentAdded( FGraphNode& node, ComponentType type )
	{
		GraphTransaction * current = _currentTransaction();

		// transactional dispatch
		if(current)
			current->addedComponents.emplace_back( node.id, type );
		// immediate dispatch
		else
		{
			auto found = _componentListeners.find( type );

			if(found == _componentListeners.end())
				return;

			for(auto listener : found->second)
				listener->componentAdded( node, type );
		}
	}

	void componentRemoved( FGraphNode& node, ComponentType type )
	{
		GraphTransaction * current = _currentTransaction();

		// transactional dispatch
		if(current)
			current->removedComponents.emplace_back( node.id, type );
		// immediate dispatch
		else
		{
			auto found = _componentListeners.find( type );

			if(found == _componentListeners.end())
			{
				for(auto listener : found->second)
					listener->componentRemoved( node, type );
			}

			componentStorage( type ).erase( node, type );
		}
	}

	void connectionAdded( int32 edgeIndex )
	{
		FGraphEdge& edge = privateEdges[edgeIndex];

		FGraphNode& nodeA = allNodes[edge.a];
		FGraphNode& nodeB = allNodes[edge.b];

		std::unordered_set<ComponentType> dispatchedTypes;

        // dispatch to typed listeners
		for(ComponentType type : nodeA.components)
		{
			auto found = _componentListeners.find( type );

			if(found == _componentListeners.end())
				continue;

			for( auto listener : found->second )
			{
				listener->connectionAdded( edgeIndex, edge, type);
				dispatchedTypes.insert( type );
			}
		}

		for(ComponentType type : nodeB.components)
		{
			auto found = _componentListeners.find( type );

			if( dispatchedTypes.find(type) != dispatchedTypes.end() || found == _componentListeners.end())
				continue;

			for(auto listener : found->second)
				listener->connectionAdded( edgeIndex, edge, type );
		}

        // dispatch to regular listeners
        for(auto listener : _edgeListeners)
            listener->connectionAdded( edgeIndex, edge );
    }

	void connectionRemoved( int32 edgeIndex )
	{
        FGraphEdge& edge = privateEdges[edgeIndex];

        FGraphNode& nodeA = allNodes[edge.a];
        FGraphNode& nodeB = allNodes[edge.b];

        std::unordered_set<ComponentType> dispatchedTypes;

        // dispatch to typed listeners
        for(ComponentType type : nodeA.components)
        {
            auto found = _componentListeners.find( type );

            if(found == _componentListeners.end())
                continue;

            for(auto listener : found->second)
            {
                listener->connectionAdded( edgeIndex, edge, type );
                dispatchedTypes.insert( type );
            }
        }

        for(ComponentType type : nodeB.components)
        {
            auto found = _componentListeners.find( type );

            if(dispatchedTypes.find( type ) != dispatchedTypes.end() || found == _componentListeners.end())
                continue;

            for(auto listener : found->second)
                listener->connectionAdded( edgeIndex, edge, type );
        }

        // dispatch to regular listeners
        for(auto listener : _edgeListeners)
            listener->connectionAdded( edgeIndex, edge );
	}

	//---------------------------------------------------------------------------
	// Simulation Registration
	//---------------------------------------------------------------------------
	template<class TObjectSimulation>
	TObjectSimulation* registerSimulation(UObject * owner)
	{
		UClass * simulationClass = TObjectSimulation::StaticClass();

		return static_cast<TObjectSimulation*>(registerSimulation( simulationClass, owner ));
	}

	UObjectSimulation * registerSimulation(TSubclassOf<UObjectSimulation> simulationClass, UObject * simulationOwner )
	{
		UObjectSimulation * simulation = nullptr;

		for(auto s : simulations)
		{
			if(s->GetClass() == simulationClass)
			{
				simulation = s;
				break;
			}
		}

		// we need to create a simulation
		if(simulation == nullptr)
		{
			simulation = NewObject<UObjectSimulation>( simulationOwner, *simulationClass );
			simulations.Add( simulation );
		}

		simulation->graph = this;

		return simulation;
	}

	template<class TObjectSimulation>
	TObjectSimulation * simulation()
	{
		for(auto s : simulations)
		{
			if(s->GetClass() == TObjectSimulation::StaticClass() )
			{
				return Cast<TObjectSimulation>(s);
			}
		}

		return nullptr;
	}

	//---------------------------------------------------------------------------
	// TStructOpsTypeTraitsBase
	//---------------------------------------------------------------------------

	FGraph& operator=( const FGraph& Other );

	// --------------------------------------------------------------
	// Serialization
	// --------------------------------------------------------------
	void PostSerialize( const FArchive& Ar ) 
	{
		// the order of component storage containers may have changed due to componentType<TComponent>()
		TArray<UScriptStruct*> derived;
		ShipEditorUtility::derivedStructs( FGraphObject::StaticStruct(), derived );

		// before trying to get the component type for these structs, we have to register them with typeForStruct
		for(UScriptStruct * scriptStruct : derived)
			FGraphObject::componentType( scriptStruct );

		// first prune any component storage that doesn't have a componentClass (the class was renamed or removed)
		{
			for( int32 i = 0; i < _componentStorage.Num(); ++i )
			{
				FComponentStorage& storage = _componentStorage[i];

				if(storage.componentClass == nullptr)
					_componentStorage.RemoveAt( i );
			}
		}

		// determine the new classes to add
		{
			std::vector<UScriptStruct*> toAdd;

			for(UScriptStruct * componentClass : derived)
			{
				FComponentStorage* found = _componentStorage.FindByPredicate( [&]( FComponentStorage& item )->bool
				{
					return item.componentClass == componentClass;
				} );

				if(found == nullptr)
					toAdd.push_back( componentClass );
			}

			// add them
			for(UScriptStruct * componentClass : toAdd)
			{
				int32 newIndex = _componentStorage.Emplace();

				_componentStorage[newIndex].componentClass = componentClass;
			}
		}

		// sort the component array by componentTypes
		// if we allocated some new component storage, we'll deal with it after, but we'll sort it to the end of the array
		_componentStorage.Sort( [&]( const FComponentStorage& a, const FComponentStorage& b ) {
			ComponentType typeA = FGraphObject::componentType( a.componentClass );
			ComponentType typeB = FGraphObject::componentType( b.componentClass );

			return typeA < typeB;
		} );

		//// for some reason, Unreal will have called PostSerialize, when only the component storage has been set on the graph, but the nodes
		//// are missing (presumably they are going to be copied, rather than serialized), in which case we are in trouble!
		//// abort, if there are no nodes
		//if(allNodes.Num() == 0)
		//	return;

		// update the graph node component arrays
		for(auto& graphNode : allNodes)
			graphNode.components.Empty();
		
		for(auto& storage : _componentStorage)
		{
			ComponentType type = FGraphObject::componentType( storage.componentClass );

			for(int i = 0; i < storage.size(); ++i)
			{
				FGraphObject * object = storage.at( i, storage.componentClass );

				FGraphNode& graphNode = node( object->nodeIndex );

				graphNode.components.AddUnique( type );
			}
		}
	}


};

template<>
struct TStructOpsTypeTraits<FGraph> : public TStructOpsTypeTraitsBase2<FGraph>
{
	enum
	{
		WithPostSerialize = false,
	};
};

// ------------------------------------------------------------
// FGraphObject implementation
// ------------------------------------------------------------

template<class TComponent, class... TArgs >
inline TComponent& FGraphNode::addComponent( FGraph& graph, TArgs... args )
{
	ComponentType type = componentType<TComponent>();

	// reinitialize the component
	if(hasComponent( type ))
	{
		TComponent& comp = component<TComponent>( graph );

		auto nodeIndex = comp.nodeIndex;

		comp = TComponent( std::forward<Args>( args )... );

		// restore the node index
		comp.nodeIndex = nodeIndex;

		return comp;
	}

	FComponentStorage& storage = graph.componentStorage( type );

	TComponent& component = storage.emplace<TComponent>( *this, std::forward<Args>( args )... );

	components.Add( type );

	graph.componentAdded( *this, type ); 

	return component;
}

inline FGraphObject* FGraphNode::addComponent( FGraph& graph, ComponentType type )
{
	FComponentStorage& storage = graph.componentStorage( type );

	UScriptStruct * typeStruct = FGraphObject::componentStruct( type );

	// reinitialize the component
	if(hasComponent( type ))
	{
		FGraphObject * comp = component( graph, type );

		auto nodeIndex = comp->nodeIndex;

		typeStruct->GetCppStructOps()->Destruct( comp );
		typeStruct->GetCppStructOps()->Construct(comp);

		// restore the node index
		comp->nodeIndex = nodeIndex;

		return comp;
	}

	FGraphObject * component = storage.emplace( *this, typeStruct );

	components.Add( type );

	graph.componentAdded( *this, type );

	return component;
}

inline FGraphObject* FGraphNode::component( FGraph& graph, ComponentType type )
{
	return graph.componentStorage(type).componentForNode( *this, type );
}

template<class TComponent>
inline TComponent& FGraphNode::component( FGraph& graph )
{
	return graph.componentStorage<TComponent>().componentForNode( *this );  
}

template<class TComponent>
inline void FGraphNode::removeComponent( FGraph& graph )
{
	ComponentType type = componentType<TComponent>();

    removeComponent( graph, type );
}

inline void FGraphNode::removeComponent( FGraph& graph, ComponentType type )
{
    FGraphObject * component = this->component( graph, type );

    component->removedFromNode( *this, graph );

	graph.componentRemoved( *this, type );

	components.Remove( type );
}

template<class TComponent>
inline bool FGraphNode::hasComponent()
{
	ComponentType type = componentType<TComponent>();

	return components.Contains( type );
}

inline bool FGraphNode::hasComponent( ComponentType type )
{
	return components.Contains( type );
}