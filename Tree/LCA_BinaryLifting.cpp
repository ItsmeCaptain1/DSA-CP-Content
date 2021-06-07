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

const int Mxn = 3e5+1 ;
const int LOG = 20 ;
int n, q ;
vector<vector<int>> up(Mxn,vector<int>(LOG)), mn(Mxn,vector<int>(LOG)) ;
vector<int> level(Mxn) ;
vector<pair<int,int>> edge[Mxn] ;



// path query on tree
// mn[j][i] = min( mn[j][i-1], mn[up[j][i-1]][i-1] ) ;
// ans = min(ans,mn[x][i] ) ; whenever x = dp[x][i]

// path update on tree
// update every edge from  node x to node y with value v 
// note that we are updating edge not vertex
// val[x] += v
// val[y] += v
// val[LCA(x,y)] -= 2*v
// we are updating every node then val[LCA(x,y)] -= v and val[parent(LCA(x,y))] -= v

// ---------------------------------------------Binary Lifting & LCA-------------------------
void dfs( int u = 1, int p = 0, int depth = 0 ){
    if( p == 0 ) up[u][0] = 1 ;
    else up[u][0] = p ; 
	level[u] = depth ;
	for( auto v : edge[u] ){
		if( v.first != p ){
			dfs(v.first,u,depth+1) ;
		}
	}
}

void binaryLifting(){
	for( int i = 1 ; i < LOG ; i++){
		for( int j = 1 ; j <= n ; j++ ){
			up[j][i] = up[up[j][i-1]][i-1] ;
            
		}
	}
}


int getKthAncestor(int node, int k) {
	if( level[node] < k )
		return -1 ;
	for( int i = LOG ; i >= 0 ; i-- ){
		int x = (1ll<<i) ; 
		if( x&k ){
			node = up[node][i] ; 
		}
	}
	return node ;
}

int LCA( int x, int y ){
	if( level[x] > level[y] ) swap(x,y) ;
	y = getKthAncestor(y,level[y]-level[x]) ; 
    if( x == y ){
		return x ; 
	}
	for( int i = LOG-1 ; i >= 0 ; i-- ){
		if( up[x][i] != up[y][i] ){
			x = up[x][i] ; 
			y = up[y][i] ;
		}	
	}
	return up[x][0] ;
}

//--------------------------------Auxiliary Tree--------------------------------
// problem on auiliary tree
// step1 -> try to figure out on O(n) tree, how we can solve this problem
// step2 -> create auxiliary tree
// step3 -> use same approach which is used in step1 

vector<int> arrival(Mxn), departure(Mxn) ; 
vector<int> edge[Mxn] ;
vector<int> auxiliaryTree[Mxn] ;

// fucntion of checking that x is a node which lie in (subtree of node k) 
// of is k is the ancestor of node x of Not 
// k == x
bool anc(int x, int k){
	return (arrival[x] >= arrival[k] ) && (departure[x] <= departure[k]) ;
}
// arrival and departure time dfs	
void arrival_departure_dfs(int u, int p = -1){
	static int t = 0 ;
	arrival[u] = ++t ;
	for( auto v : edge[u] ){
		if( p != v.first ) arrival_departure_dfs(v.ff,u) ;
	}
	departure[u] = ++t ;
}

bool cmp(const int& l, const int& r){
	return arrival[l] < arrival[r] ;
}

int createAuxiliaryTree(vector<int>& q){
	sort(all(q),cmp) ;
	int k = q.size(); 
	for( int i = 1 ; i < k ; i++ ){
		q.pb(LCA(q[i],q[i-1])) ; 
	}
	sort(all(q),cmp) ; 
	q.resize(distance(q.begin(),unique(q.begin(),q.end()))) ;

	// build tree
	stack<int> s ; 
	s.push(q[0]) ; 
	for( int i = 1 ; i < q.size() ; i++ ){
		while( !anc(q[i],s.top()) ) s.pop() ; 
		auxiliaryTree[s.top()].pb(q[i]) ;
		auxiliaryTree[q[i]].pb(s.top()) ; 
		s.push(q[i]) ;	
	}
	return q[0] ;
}

int subsize[Mxn]; 
ll total_distance ;

void dfs(int node,int cost,int parent)
{
	subsize[node]=1; 
	for(auto it : edge[node])
	{
		int next=it.first;
		int c=it.second;
		if(next!=parent)
		{
			dfs(next,c,node);
			subsize[node]+=subsize[next]; 
		}
	}
	total_distance += cost*(n-subsize[node])*subsize[node] ;
}

void solve(){
    cin >> n >> q ; 
    for( int i = 2 ; i <= n ; i++ ){
        int a; scanf("%d",&a) ;
        edge[a].pb({i,0}) ; 
        edge[i].pb({a,0}) ; 
    }
    dfs() ;
    binaryLifting() ;
    while(q--){
        int a, b ; scanf("%d%d",&a,&b) ; 
        int x = LCA(a,b) ; 
        cout << x << endl ;
    }
}


int main(){
	int t = 1 ;

	//clock_t start , finish ;
	//start = clock() ;
	while(t--){
		solve() ;
	}
	//finish = clock() ;
	//cout << fixed << setprecision(3) << (float)(finish - start)/CLOCKS_PER_SEC ;
	
}

