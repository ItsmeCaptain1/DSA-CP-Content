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
const int Mxn = 2e5+5 ;

vector<ll> pow2;
ll n, q ;

struct item{
    ll p1, p2, p3 ; 
};
vector<ll> a ;
vector<item> segtree ;

void build(){
    segtree.resize(2*n,{0,0,0}) ;
}

ll pointQuery( ll pos ){
    pos += n ; 
    ll res = segtree[pos].p1  ; 

    int last = pos ; 
    pos /= 2 ;
    ll mul = 0, add = 1 ; 
    if( pos*2 != last ){
        mul += add ;
    }
    while( pos >= 1 ){
        res += segtree[pos].p1 + segtree[pos].p2*mul+ segtree[pos].p3*mul*mul ; 
        add *= 2ll ;
        last = pos ;
        pos >>= 1 ;
        if( pos*2 != last ){
            mul += add ;
        }
    }
    return res ; 
}

void rangeUpdate(ll l, ll r, ll val, ll id = 1, ll leftLimit = 0, ll rightLimit = n-1 ){
    if( leftLimit > rightLimit || r < leftLimit || l > rightLimit ) return ; 
    
    if( l <= leftLimit && rightLimit <= r ){
        segtree[id].p1 += val*val ;
        segtree[id].p2 += 2ll*val ;
        segtree[id].p3 += 1 ;
        return ; 
    }
    ll mid = (leftLimit+rightLimit)>>1 ;
    int ex = 0 ;
    if( l <= mid && r > mid ){
        ex = (mid-l+1) ;
    }
    rangeUpdate(l,r,val,2*id,leftLimit,mid) ;
    rangeUpdate(l,r,val+ex,2*id+1,mid+1, rightLimit) ;
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
        }
        else{
            ll pos; cin >> pos, pos-- ; 
            cout << a[pos] + pointQuery(pos) << endl ;
        }
    }




}

int main(){
	ios_base::sync_with_stdio(false);cin.tie(NULL);
	int t = 1  ;
    for( int i = 0 ; i <= 20 ; i++ ){
        pow2.pb(i<<1) ;
    }
	//clock_t start , finish ;
	//start = clock() ;
	while(t--){
		solve() ;
	}
	//finish = clock() ;
	//cout << fixed << setprecision(3) << (float)(finish - start)/CLOCKS_PER_SEC ;
	
}

