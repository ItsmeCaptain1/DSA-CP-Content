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
vector<int> edge[Mxn] ;
vector<int> vis(Mxn,false) ;

int dfsForTopo(int u, vector<int>& ordering, int& i){
    vis[u] = true ; 

    for( int v : edge[u] ){
        if( vis[v] == false ){
            i = dfsForTopo(v,ordering,i) ;
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

void solve(){

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

