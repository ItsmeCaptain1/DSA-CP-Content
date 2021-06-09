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

// Bellman Ford Algo
// sssp with -ve edge 
// TC = E*V 

vector<ll> bellmanford(int start){
    vector<ll> dist(n,INT_MAX) ; 
    dist[start] = 0 ; 

    for( int i = 0 ; i < n-1 ; i++ ){
        for( int j = 0 ; j < n ; j++ ){
            for( auto v : edge[j] ){
                if( dist[v.ff] < dist[j] + v.ss ){
                    dist[v.ff] = dist[j] + v.ss ;
                }
            }
        }
    }
    for( int i = 0 ; i < n-1 ; i++ ){
        for( int j = 0 ; j < n ; j++ ){
            for( auto v : edge[j] ){
                if( dist[v.ff] < dist[j] + v.ss ){
                    dist[v.ff] = INT_MIN ;
                }
            }
        }
    }
    return dist ;
}

// Floyed Warshall algo 
// ALL pair shortest path
vector<vector<int>> dp, nxt ; 

vector<vector<int>> floyedwarshall(vector<vector<int>>& mat){
    dp = vector<vector<int>> (n,vector<int> (n)) ;
    nxt = vector<vector<int>> (n,vector<int> (n)) ; 

    for( int i = 0 ; i < n ; i++ ){
        for( int j = 0 ; j < n ; j++ ){
            dp[i][j] = mat[i][j] ; 
            if( mat[i][j] != INT_MAX ){
                nxt[i][j] = j ; 
            }
        }
    }

    for( int k = 0 ; k < n ; k++ ){
        for( int i = 0 ; i < n ; i++ ){
            for( int j = 0 ; j < n ; j++ ){
                if( dp[i][j] > dp[i][k] + dp[k][j] ){
                    dp[i][j] = dp[i][k] + dp[k][j] ;
                    nxt[i][j] = nxt[i][k] ;
                }
            }
        }
    }
    // detect and propogate -ve cycles 
    // for( int k = 0 ; k < n ; k++ ){
    //     for( int i = 0 ; i < n ; i++ ){
    //         for( int j = 0 ; j < n ; j++ ){
    //             if( dp[i][j] > dp[i][k] + dp[k][j] ){
    //                 dp[i][j] = INT_MIN ;
    //                 nxt[i][j] = -1 ; 
    //             }
    //         }
    //     }
    // }

    return dp ;
}

vector<int> reconstructPath(int start, int end ){
    vector<int> path ; 
    if( dp[start][end] == INT_MAX ) return path ; 
    int at = start ; 

    for(  ; at != end ; at = nxt[at][end] ){
        if( at == -1 ){
            return {-1} ;
        }
        path.pb(at) ;
    }
    if( nxt[at][end] == -1  ){
        return {-1} ;
    }
    path.pb(end) ;
    return path ; 
}

// Articulation Points and Bridges
// in a bridge u or v is articulation point 
// but articulation points always don't have to be on bridge they can exits without bridges
//  articulation points must have more then 1 outgoing edges    
int id = 0 ;
vector<int> ids(Mxn), low(Mxn) ; 
vector<bool> vis(Mxn), isArt(Mxn) ;

int outEdgecount = 0 ;

void dfs(int root, int u, int pr){
    if(root==pr) outEdgecount++ ; 
    vis[u] = true ; 
    id++ ; 
    low[u] = ids[u] = id ;

    for( auto v : edge[u] ){
        if( v.ff == pr ) continue ; 
        if( !vis[v.ff] ){
            dfs(root,v.ff,u) ;
            low[u] = min(low[u], low[v.ff]) ; 
            // Art Point found via bridges  
            if( ids[u] < low[v.ff] ){
                isArt[u] = true ; 
            }
            // Art point found via cycle
            if( ids[u] ==low[v.ff] ){
                isArt[u] = true ; 
            }
        }else{
            low[u] = min(low[u], ids[v.ff]) ; 
        }
    }

}

vector<bool> findArt(){
    for( int i = 0 ; i < n ; i++){
        if( !vis[i] ){
            outEdgecount = 0; 
            dfs(i,i,-1) ; 
            isArt[i] = (outEdgecount>1); 
        }
    }
    return isArt ;
}

void DFS(int u, int pr, vector<int>& bridges ){
    vis[u] = true ; 
    id++ ; 
    low[u] = ids[u] = id ;

    for( auto v : edge[u] ){
        if( v.ff == pr ) continue ; 
        if( !vis[v.ff] ){
            DFS(v.ff,u,bridges) ;
            low[u] = min(low[u], low[v.ff]) ; 
            if( ids[u] < low[v.ff] ){
                bridges.pb(u) ;
                bridges.pb(v.ff) ;
            }
        }else{
            low[u] = min(low[u], ids[v.ff]) ; 
        }
    }

}

vector<int> findBridges(){
    vector<int> bridges ;
    for( int i = 0 ; i < n ; i++ ){
        if( !vis[i] ){
            DFS(i,-1,bridges) ; 
        } 
    }
    return bridges ;
}

// Tarjan's Algorithm
// for Strongly connected component 

const int unvisited = -1 ;
int ID = 0, sccCount = 0 ;
vector<int> ids, low ;
vector<bool> onStack ; 
stack<int> st ;  

void dfs(int at){
    st.push(at) ; 
    ids[at] = low[at] = ID++ ; 
    onStack[at] = true ; 

    for( auto v : edge[at] ){
        if( ids[v.ff] == unvisited ) dfs(v.ff) ; 
        if( onStack[v.ff] ) low[at] = min(low[at], low[v.ff]) ; 
    }

    if( low[at] == ids[at] ){
        int node = st.top() ; 
        onStack[node] = false ;
        st.pop() ; 
        while( node != at ){
            node = st.top() ; 
            st.pop() ; 

            onStack[node] = false ;
            low[node] = ids[at] ;
        }
        sccCount++ ; 
    } 
}

vector<int> tarjanAlgo(){
    onStack = vector<bool> (n,false) ;
    ids = vector<int> (n,unvisited) ;
    low = vector<int> (n,0) ;

    for( int i = 0 ; i < n ; i++ ){
        if( ids[i] == unvisited ){
            dfs(i) ;
        } 
    }
    return low ;
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

