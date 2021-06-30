#include<bits/stdc++.h>
using namespace std ;

#define ll long long
#define pb push_back
#define all(v) v.begin(),v.end()
#define ff first
#define ss second
#define ar array
#define endl '\n'
#define trace(x) cerr << #x << " = " << x << endl ;
const int mod = 1e9+7 ;
const int Mxn = 2e5+5 ;

ll n, q ;

struct item{
    ll p1, p2, p3 ; 
    
};
vector<ll> a ;
vector<item> segtree, lazy ;

void build(){
    segtree.resize(2*n,{0,0,0}) ;
    lazy.resize(2*n,{0,0,0}) ;
    // for( ll i = n ; i < 2*n; i++ ){
    //     segtree[i] = a[i-n] ; 
    // }

    // for( ll i = n-1 ; i > 0 ; i-- ){
    //     segtree[i] = min(segtree[2*i],segtree[2*i+1]) ; 
    // }
}

void combine(ll id, ll sz){
    segtree[id].p1 += sz*lazy[id].p1 ;
    segtree[id].p2 += sz*lazy[id].p2 ;
    segtree[id].p3 += sz*lazy[id].p3 ;
}

item add(item x, item y){
    return { x.p1+y.p1, x.p2+y.p2, x.p3+y.p3} ;
}

void push(ll id, ll lo,ll hi){
    if( lazy[id].p1 != 0 ){
        combine(id,hi-lo+1) ;  
        if( lo != hi ){
            lazy[2*id] = add(lazy[2*id], lazy[id]) ;
            lazy[2*id+1] = add(lazy[2*id+1], lazy[id]) ;
        }
        lazy[id] = {0,0,0} ;
    }
}


item rangeQuery(ll l, ll r, ll id = 1, ll leftLimit = 0, ll rightLimit = n-1 ){
        push(id,leftLimit,rightLimit) ;
    if( rightLimit < l || leftLimit > r ){
        return {0,0,0}; 
    }
    if( l <= leftLimit && rightLimit <= r ){
        return segtree[id] ; 
    }
    ll mid = (leftLimit + rightLimit)>>1 ; 
    item x = rangeQuery(l,r,2*id, leftLimit, mid ); 
    item y = rangeQuery(l,r,2*id+1, mid+1, rightLimit) ; 
    return add(x,y) ; 

}


// void pointUpdate(int pos, ll val ){
//     segtree[pos+=n] = val ;
//     for(  pos/=2 ; pos >= 1 ; pos/= 2){
//         segtree[pos] = min(segtree[2*pos],segtree[2*pos+1] );
//     }
// }

// void pointUpdateRecursively(ll pos, ll val, int id = 1, ll leftLimit = 0, ll rightLimit = n-1 ){
//     if( leftLimit == pos && pos == rightLimit ){
//         // I'm leaf ; 
//         segtree[id] = val ; 
//         return ;
//     }
//     if( pos > rightLimit || pos < leftLimit ){
//         return ; 
//     }
//     ll mid = ( leftLimit + rightLimit ) / 2 ; 
//     pointUpdateRecursively(pos, val, 2*id, leftLimit, mid) ; 
//     pointUpdateRecursively(pos, val, 2*id+1, mid+1, rightLimit) ; 
//     segtree[id] = segtree[2*id] + segtree[2*id+1] ;
// }

void rangeUpdate(ll l, ll r, ll x ,ll id = 1, ll leftLimit = 0, ll rightLimit = n-1 ){
    push(id,leftLimit,rightLimit) ;
    if( leftLimit > rightLimit || r < leftLimit || l > rightLimit ) return ; 
    if( l <= leftLimit && rightLimit <= r ){
        // trace(x);
        lazy[id].p1 += x*x + l*l - 2*l*x  ;
        lazy[id].p2 += 2*x - 2*l ;
        lazy[id].p3 += 1 ;
        push(id,leftLimit,rightLimit) ;
        return ; 
    }
    ll mid = (leftLimit+rightLimit)>>1 ;
    
    rangeUpdate(l,r,x,2*id,leftLimit,mid) ;
    rangeUpdate(l,r,x,2*id+1,mid+1, rightLimit) ;
    segtree[id] = add(segtree[2*id],segtree[1+2*id]) ;
}

void solve(){ 
    cin >> n >> q;
    a.resize(n) ;
    for( int i = 0 ; i < n; i++ ){
        cin >> a[i]; 
    }
    // n converting to power of 2 
    while( __builtin_popcount(n) != 1 ){
        n++ ; 
        a.pb(0) ; 
    }
    build() ;
    
    while( q-- ){
        int type ; cin >> type ; 
        if( type == 1 ){
            ll l,r, x ; cin >> l >> r >> x, l--, r-- ; 
            rangeUpdate(l,r,x);
            // x = 1; int y = 1 ;
            // for( int i = 1 ; i < 2*n ;i++ ){
            //     cout << segtree[i].p1 << " " << segtree[i].p2 << " " << segtree[i].p3 << " ---- " ;
            //     if( x == 1 ){
            //         cout << endl ;  
            //         x = y*2 ; 
            //         y = x ; 
            //     }else{
            //         x-- ; 
            //     }
            // }
        }
        else{
            ll pos; cin >> pos, pos-- ; 
            item res = rangeQuery(pos,pos) ;
            cout << a[pos] + res.p1 + res.p2*pos + res.p3*pos*pos << endl ;
        }
    }




}

int main(){
	ios_base::sync_with_stdio(false);cin.tie(NULL);
	int t = 1  ;
	//clock_t start , finish ;
	//start = clock() ;
	while(t--){
		solve() ;
	}
	//finish = clock() ;
	//cout << fixed << setprecision(3) << (float)(finish - start)/CLOCKS_PER_SEC ;
	
}

