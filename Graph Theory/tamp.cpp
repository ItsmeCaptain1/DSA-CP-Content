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
            if( dist[v.ff]  > v.ff + dist[u.ss] ){
                prev[v.ff] = u.ss ;
                dist[v.ff] = v.ff + dist[u.ss] ;
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


// Eulerian Path or Eulerian Trail -> visits all the edges in a graph exactly onces 
// Eulerian Circuit -> is EP which start and end on same node 
// Not every graph have eulerian path 
// Also we have to care about from which node we are starting traverse

/*
                   Eulerian Circuit                Eulerian Path 

Undirected       Every vertex has an         every vertex has an even degree or 
Graph                 even degree            exactly two vertices have odd degree

Directed         Every vertex has equal      at most one vertex has (out-in) = 1 and  
Graph           IN-degree and OUT degree    at most one vertex has (in-out) = 1 and 
                                            all other vertices have in == out 

        Indirected graph -> graph should be single connected component of EP and EC 

*/

void countInOutDegrees(vector<int>& in, vector<int>& out){
    for( int i = 1 ; i <= n ; i++ ){
        for( auto v : edge[i] ){
            in[i]++ ; 
            out[v.ff]++ ; 
        }
    }
}

bool graphHasEulerianPath(vector<int>& in, vector<int>& out){
    int start_nodes = 0, end_nodes = 0;
    for( int i = 1 ; i<= n ; i++ ){
        if( (out[i]-in[i]) > 1 || (in[i]-out[i]) > 1 ){
            return false ; 
        }
        else if( (out[i]-in[i]) == 1 ){
            start_nodes++ ; 
        }else if( (in[i]-out[i]) == 1 ){
            end_nodes++ ;
        }
    }
    return (start_nodes==0&&end_nodes==0) || (start_nodes==1&&end_nodes==1) ;
}

int startNode(vector<int>& in, vector<int>& out){
    int start = 1 ;
    for( int i = 1 ; i <= n ; i++ ){
        if( (out[i]-in[i]) == 1 ){
            return i ; 
        }
        if( out[i] > 0 ) start = i ; 
    }
    return start ; 
}

void dfs( int u, vector<int>& out, vector<int>& path){
    while(out[u] != 0){
        int v = edge[u][--out[u]].ff ; 
        dfs(v,out,path) ;
    }
    path.pb(u) ;
}
int m ;
vector<int> eulerianPath(){
    vector<int> in(n), out(n), path ; 
    countInOutDegrees(in,out) ;
    if( !graphHasEulerianPath(in, out) ) return {} ;

    dfs(startNode(in,out),out,path) ; 
    if( path.size() == m+1 ) return path ;// m is number of edge
    return {} ; 
}






// MST 
// Prim's MST algo 

struct item{
    int u, v, c ;
    bool operator<(const item& other) const {
        return c > other.c ; 
    }

};

void addEdge(int s, priority_queue<item>& pq,vector<bool>& vis ){
    vis[s] = true ; 
    for( auto v : edge[s] ){
        if( vis[v.ff] != false ){
            pq.push({s,v.ff,v.ss}) ; 
        }
    }
}

int MST(int s = 0 ){
    vector<bool> vis(n,false) ; 
    int m = n-1 ;  // number of expected edges in MST
    int edgeCount = 0, mstCost = 0 ; 
    vector<item> mstEdge(m) ;

    priority_queue<item> pq ;
    addEdge(s,pq,vis) ;  
    vis[s] = true ; 

    while( !pq.empty() ){
        item u = pq.top() ; 
        pq.pop() ; 
        if( vis[u.v] ) continue ; 
        mstEdge[edgeCount++] = u ; 
        mstCost += u.c ; 

        addEdge(u.v,pq,vis) ; 
    }
    if( edgeCount != m ) return INT_MIN ; 
    return mstCost ;  
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


// a^(-1) % mod = a^(m-2) % mod 

// 


//  ---------------------------          tree diameter         --------------------
// find farthest node x from node i (random), then find farthest node y from node x
// distance between node x and node y is diameter of tree
// for finding middle element ( d = distance of diameter )
// ( distx[v] == (d/2) && disty[v] == (d - (d/2))) 










// -------------------------------          rerooting    // 5-6 star topic      ----------------

// let's start with randome node ( let's say node 1 )
// we always calculate 2 things first subtreesize and sum for each node
// sometime dp is also helpful dp = sum + sub 
// sum[u] = (all sum[v])  + sub[u] - 1( this is distfrom root node :) ) 
// 
// in-out dp -> in[u] + out[u] = sum[u], so it will be nice if we calculate out instead of sum[u]
// 
// void dfs(int u, int p){
//     sub[u] = 1 ; 
//     sum[u] = 0 ; 
//     for( int v : edge[u] ){
//         if( v == p ) contiue ; 
//         sub[u] += sub[v] ;
//         sum[u] += sum[v] ;
//     }
//     sum[u] += (sub[u]-1) ;
//     dp[u] = sub[u] + sum[u] ; 
// }

/* here we start doing rerooting proces
    void rerooting(int u, int p ){
        res[u] = dp[u] or sum[u] or somthing else mixed of these ; 
        for( int v : edge[u] ){
            if( v == p ) continue ; 
            
            // for rooting node v there are two steps 

            // first adjust sub and sum for node u ( converting root to child )
            sub[u] -= sub[v] ; 
            sum[u] -= (sub[v]+sum[v]) ;
            //  if using dp then bcoz dp is combo of sub and sum so 
            dp[u] -= ( sub[v] + (sub[v]+sum[v]) ) ; 
            // let's calculate out part 
            out[v] = out[u] + ( in[u] - sub[v] - in[v] ) + ( n - sub[v] ) ;

            // 2nd step adjust node v ( converting child to root )
            sum[v] += (sub[u] + sub[u]) ;
            sub[v] += sub[u] 
            // for dp adjustment 
            dp[v] += sub[u] + ( dp[u] or ( sub[u] + sum[u] ) ) 
            // ( dp[u] or ( sub[u] + sum[u] ) )  bcoz dp is made of sub + sum :)
            
            dfs(v,u) ;

            // Die soon bro :)
        }
    }
*/










//------------------------------ Histogram ( maximum rectangle ) ------------------
/*
int maxHist(vector<int> row)
{
    // Create an empty stack. 
    // The stack holds indexes of
    // hist[] array/ The bars stored 
    // in stack are always
    // in increasing order of their heights.
    stack<int> result;

    int top_val; // Top of stack

    int max_area = 0; // Initialize max area in current
    // row (or histogram)

    int area = 0; // Initialize area with current top

    // Run through all bars of given histogram (or row)
    int i = 0;
    while (i < row.size()) {
        // If this bar is higher than the bar on top stack,
        // push it to stack
        if (result.empty() || row[result.top()] <= row[i])
            result.push(i++);

        else {
            // If this bar is lower than top of stack, then
            // calculate area of rectangle with stack top as
            // the smallest (or minimum height) bar. 'i' is
            // 'right index' for the top and element before
            // top in stack is 'left index'
            top_val = row[result.top()];
            result.pop();
            area = top_val * i;

            if (!result.empty())
                area = top_val * (i - result.top() - 1);
            max_area = max(area, max_area);
        }
    }

    // Now pop the remaining bars from stack and calculate
    // area with every popped bar as the smallest bar
    while (!result.empty()) {
        top_val = row[result.top()];
        result.pop();
        area = top_val * i;
        if (!result.empty())
            area = top_val * (i - result.top() - 1);

        max_area = max(area, max_area);
    }
    return max_area;
}
*/











