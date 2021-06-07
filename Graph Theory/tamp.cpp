#include<bits/stdc++.h>
using namespace std ;

#define ll long long
#define pb push_back
#define all(v) v.begin(),v.end()
#define ff first
#define ss second
#define ar array
#define endl '\n'
const int mod = 1e9+7 ;

int dx[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };
int dy[8] = { 1, 0, -1, 0, 1, -1, 1, -1 };

// # Bipartite Graph -> There is no odd length cycle 
// # complete Graph -> there ia unique adge between every pair of node (n*(n-1))/2
// topological ordring are only for DAGs
// topological ordring are not unique bcoz we can start performing topsort from any node 
const int Mxn = 2e5+2 ;
int n;
vector<pair<int,int>> edge[Mxn] ;
vector<int> vis(Mxn,false) ;

int dfsForTopo(int u, vector<int>& ordering, int i){
    vis[u] = true ; 

    for( auto v : edge[u] ){
        if( vis[v.ff] == false ){
            i = dfsForTopo(v.ff,ordering,i) ;
        }
    } 
    ordering[i] = u ; 
    return i-1 ;   
}

vector<int> topologicalOrdring(){
    vector<int> ordering(n) ; 
    int i = n-1 ;

    for( int at = 0 ; at < n ; at++ ){
        if( !vis[at] ){
            dfsForTopo(at,ordering,i) ;
        }
    }
    return ordering ; 
}

// we will get distance vector in which distance are respected to startnode and calculated using edge weight

// TC = O(V+E)
// no restriction except graph must be DAG -> Directed Acyclic Graph

vector<int> DAG_shortest_path(int startNode){
    vector<int> topsort, dist(n,INT_MAX) ; 
    topsort = topologicalOrdring() ;
    dist[startNode] = 0 ; 

    for( int i = 0 ; i < n ; i++ ){
        int nodeIndex = topsort[i] ; 
        if( dist[nodeIndex] != INT_MAX ){
            for( auto v : edge[nodeIndex] ){
                int newDist = dist[nodeIndex] + v.ss; 
                dist[v.ff] = min( dist[v.ff], newDist ) ; 
            }
        }
    } 
    return dist ;   
}

// Dijkstra's Algorithm
// Graph must only contain non-neg edge that's how we work like greedy algo
// TC = O(ElogV)
// The Game plan behind this algo is like if i visit any node first time, 
// that is gonna be shortest distance for that node from start node. 
// Dijkstra'a Algo proccess each next most promising node in order.
// use endNode for finding shortest path from startNode to endNode, else ignore the endNode.

// ways to improve Dijsktra's algo
// 1. Indexed Priority Queue (IPQ)
// 2. Fibonacci heap -> TC = O( E + VlogV )

vector<int> dijkstra(int startNode, int endNode = -1){
    vector<int> dist(n,INT_MAX), prev(n,-1) ; 
    dist[startNode] = 0 ;

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>>pq ; 
    pq.push({0,startNode}) ; 

    while(!pq.empty()){
        pair<int,int> u = pq.top(); 
        pq.pop() ;  
        vis[u.ss] = true ; 

        if( dist[u.ss] < u.ff ) continue ; 

        for( auto v : edge[u.ss] ){
            if(vis[v.ff]) continue ;
            if( dist[v.ff]  > u.ff + dist[u.ss] ){
                prev[v.ff] = u.ss ;
                dist[v.ff] = u.ff + dist[u.ss] ;
                pq.push({dist[v.ff],v.ff}) ;
            }
        }
        // for finding distance from 1 to 1 node 
        // if( endNode == u.ss ){
        //     return dist[u.ss] ;
        // }
    }
    //  return dist ;

    vector<int> path ; 
    if( dist[endNode] == INT_MAX ) return path ;
    for( int at = endNode ; at != -1 ; at = prev[at] ){
        path.pb(at) ;
    }
    reverse(all(path)) ;
    return path ; 

}




void solve(){
    cin >> n ;
}


int main(){
	int t ; cin >> t ;

	//clock_t start , finish ;
	//start = clock() ;
	while(t--){
		solve() ;
	}
	//finish = clock() ;
	//cout << fixed << setprecision(3) << (float)(finish - start)/CLOCKS_PER_SEC ;
	
}

