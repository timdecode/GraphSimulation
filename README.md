# GraphSimulation

This is a graph-based entity-component-system for Unreal Engine 4. `Nodes` are like entities, `GraphObjects` are like components. `Simulations` enumerate `GraphObjects`. Nodes can maintain links to other nodes with the framework correctly updating links when nodes are removed.

Features
- simple
- `GraphObjects` are structs stored in packed arrays, they are efficient and cache-friendly to tick in bulk
- uses `UInstancedStaticMeshComponents` to render vast groups of entities at 90 fps
- integrates with Unreal's serialization system
- transactions for creating and deleting `Nodes` and `GraphObjects` (this makes managing the node-graph easy to use and efficient for `Simulations` to deal with)

This code compiles and works correctly, however I have not converted this repository into a plugin yet. This will come in a later update.
