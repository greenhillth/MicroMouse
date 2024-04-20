#include "Graph.hpp"
#include "LinkedList.hpp"
// #include <iostream>

namespace mtrn3100 {

// // COMPLETE THIS FUNCTION.
// template <typename N, typename E>
// bool bfs_simple(Graph<N, E> const& g, N const& src, N const& dst) {
//     LinkedList<N> queue;
//     LinkedList<N> visited;
//     queue.push_front(src);
//     while(queue.size() > 0){
//         int n = queue.pop_front();
//         if(n == dst){
//             return true;
//         }
//         if(!visited.contains(n)){
//             visited.push_back(n);
//             LinkedList<N> new_nodes = g.nodes(n);
//             for(int i=0; i < new_nodes.size(); i++){
//                 queue.push_back(new_nodes.get(i)->value);
//             }
//         }

//     }
//     return false;
// }

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
LinkedList<N> bfs_single(Graph<N, E> const& g, N const& src, N const& dst) {
    LinkedList<N> queue;
    LinkedList<N> visited;
    LinkedList<N> final_path;
    int path[g.size()+1];
    int pSize = (g.size()+1);
    for (int k = 0; k < pSize; k++) {
        path[k] = -1;
    }
    path[src] = src;

    queue.push_front(src);
    visited.push_back(src);
    while(!queue.empty()){
        int n = queue.pop_front();
        if(n == dst){
            
            int index = n;
            
            while(path[index] != index){
                
                final_path.push_front(index);
                index = path[index];
                
            }
            final_path.push_front(index);
            return final_path;
        }
        LinkedList<N> adj = g.nodes(n);
        for(int i = 0; i < adj.size(); i++){
            if(!visited.find(adj.get(i)->value)){
                queue.push_back(adj.get(i)->value);
                visited.push_back(adj.get(i)->value);
                // std::cout<<n <<  ",  " <<  adj.get(i)->value  <<std::endl;
                path[adj.get(i)->value] = n;
            }
        }

    }
    return final_path;
}

// // COMPLETE THIS FUNCTION.
// template <typename N, typename E>
// LinkedList<LinkedList<N>> bfs_multiple(Graph<N, E> const& g, N const& src, N const& dst) {
//     LinkedList<LinkedList<N>> paths;
//     //LinkedList<LinkedList<N>> queue;
//     //queue.push_front({src});
//     LinkedList<N> queue;
//     LinkedList<N> visited;
//     LinkedList<N> final_path;
//     int path[g.size()+1];
//     std::fill (path,path+g.size()+1,-1);
//     path[src] = src;
//     int path_found = 0;

//     queue.push_front(src);
//     visited.push_back(src);
//     while(!queue.empty() || path_found){
//         int n = queue.pop_front();
//         if(path_found){
           
//             int index = path_found;
//             path_found = 0;
//             final_path.push_front(dst);
//             while(path[index] != index){
                
//                 final_path.push_front(index);
//                 index = path[index];
                
//             }
//             // std::cout << final_path.size() << std::endl;
//             final_path.push_front(index);
//             paths.push_back(final_path);
//             while(!final_path.empty()){
//                 // std::cout << "path:" << final_path.pop_back() << std::endl;
//             }

//         }
//         LinkedList<N> adj = g.nodes(n);
//         for(int i = 0; i < adj.size(); i++){
            
//             if(!visited.find(adj.get(i)->value)){
//                 if(!path_found){
                    
//                 }
//                 queue.push_back(adj.get(i)->value);
//                 if(adj.get(i)->value != dst){
//                     visited.push_back(adj.get(i)->value);
//                     path[adj.get(i)->value] = n;
                    
//                 }else{
//                     //std::cout<<"path found" <<std::endl;
//                     path_found = n;
//                 }
//                 //std::cout<<path_found<< ",  " << n <<  ",  " <<  adj.get(i)->value  <<std::endl;
                
//             }
//         }

//     }

//     if(paths.size()>0){
//         int min = paths.get(0)->value.size();
//         for(int i = 1; i<paths.size();i++){
//             if(min>paths.get(i)->value.size()){
//                 min = paths.get(i)->value.size();
//             }
//         }
//         for(int i = 0; i<paths.size();i++){
//             //std::cout << min << ",  " << paths.get(i)->value.size() <<std::endl;
//             if(min<paths.get(i)->value.size()){
//                 paths.remove(i);
//             }
//         }
//     }
//     return paths;
// }

}  // namespace mtrn3100
