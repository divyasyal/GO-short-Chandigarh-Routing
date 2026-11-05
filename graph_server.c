/* graph_server.c
 * Minimal shortest-path HTTP server for demo.
 * Compile (MSYS2): gcc -O2 graph_server.c -o graph_server -lm -lws2_32
 *
 * Usage: ./graph_server nodes.csv edges.csv
 *
 * Endpoints:
 *   GET /nodes
 *     -> returns GeoJSON FeatureCollection of nodes
 *   GET /shortest?src=<nodeid>&dst=<nodeid>
 *     -> returns JSON { path: [ {id,lat,lon}, ... ], distance: meters }
 *
 * Note: nodes.csv format: id,lat,lon
 *       edges.csv format: from,to,weight,wayid
 *
 * CORS header included for browser access.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>
#include <stdbool.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#define PORT 8080
#define BUFSIZE 65536

typedef struct {
    long long id;
    double lat, lon;
} Node;

typedef struct Edge {
    int to;
    double w;
    struct Edge *next;
} Edge;

typedef struct {
    Node *nodes;
    size_t n_nodes;
    Edge **adj;
} Graph;

/* Haversine distance (meters) */
static inline double deg2rad(double d){ return d * M_PI / 180.0; }
double haversine(double lat1,double lon1,double lat2,double lon2){
    double R = 6371000.0;
    double dlat = deg2rad(lat2-lat1);
    double dlon = deg2rad(lon2-lon1);
    double a = sin(dlat/2.0)*sin(dlat/2.0) + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dlon/2.0)*sin(dlon/2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;
}

/* Map id -> index (linear search) */
size_t find_index(Graph *g, long long id){
    for(size_t i=0;i<g->n_nodes;i++) if(g->nodes[i].id==id) return i;
    return (size_t)-1;
}

/* Load CSV graph */
Graph *load_graph(const char *nodes_csv,const char *edges_csv){
    FILE *f=fopen(nodes_csv,"r");
    if(!f){ perror("nodes.csv"); return NULL; }
    char line[4096];
    size_t count=0;
    while(fgets(line,sizeof(line),f)) if(strchr(line,',')) count++;
    rewind(f);

    Node *nodes=calloc(count,sizeof(Node));
    size_t idx=0;
    while(fgets(line,sizeof(line),f)){
        if(strchr(line,',')==NULL) continue;
        long long id; double lat, lon;
        if(sscanf(line,"%lld,%lf,%lf",&id,&lat,&lon)>=3){
            nodes[idx].id=id;
            nodes[idx].lat=lat;
            nodes[idx].lon=lon;
            idx++;
        }
    }
    fclose(f);

    Graph *g=calloc(1,sizeof(Graph));
    g->nodes=nodes;
    g->n_nodes=idx;
    g->adj=calloc(g->n_nodes,sizeof(Edge*));

    f=fopen(edges_csv,"r");
    if(!f){ perror("edges.csv"); return NULL; }
    while(fgets(line,sizeof(line),f)){
        if(strchr(line,',')==NULL) continue;
        long long fromid,toid; double w; int wayid=0;
        if(sscanf(line,"%lld,%lld,%lf,%d",&fromid,&toid,&w,&wayid)<3) continue;
        size_t fi=find_index(g,fromid);
        size_t ti=find_index(g,toid);
        if(fi==(size_t)-1 || ti==(size_t)-1) continue;
        Edge *e=malloc(sizeof(Edge));
        e->to=(int)ti;
        e->w=w;
        e->next=g->adj[fi];
        g->adj[fi]=e;
    }
    fclose(f);
    return g;
}

/* Min-heap for Dijkstra */
typedef struct { int node; double dist; } HN;
typedef struct {
    HN *a; int size; int cap; int *pos;
} MinHeap;

MinHeap *heap_create(int n){
    MinHeap *h=malloc(sizeof(MinHeap));
    h->cap=n+5;
    h->a=malloc(sizeof(HN)*(h->cap));
    h->size=0;
    h->pos=malloc(sizeof(int)*n);
    for(int i=0;i<n;i++) h->pos[i]=-1;
    return h;
}

void heap_swap(MinHeap *h,int i,int j){
    HN t=h->a[i]; h->a[i]=h->a[j]; h->a[j]=t;
    h->pos[h->a[i].node]=i;
    h->pos[h->a[j].node]=j;
}

void heap_up(MinHeap *h,int i){
    while(i>0){
        int p=(i-1)/2;
        if(h->a[p].dist<=h->a[i].dist) break;
        heap_swap(h,p,i); i=p;
    }
}

void heap_down(MinHeap *h,int i){
    int n=h->size;
    while(1){
        int l=2*i+1,r=2*i+2,s=i;
        if(l<n && h->a[l].dist<h->a[s].dist) s=l;
        if(r<n && h->a[r].dist<h->a[s].dist) s=r;
        if(s==i) break;
        heap_swap(h,i,s); i=s;
    }
}

void heap_push_or_dec(MinHeap *h,int node,double d){
    if(h->pos[node]!=-1){
        int i=h->pos[node];
        if(d<h->a[i].dist){ h->a[i].dist=d; heap_up(h,i); }
    } else {
        int i=h->size++;
        h->a[i].node=node;
        h->a[i].dist=d;
        h->pos[node]=i;
        heap_up(h,i);
    }
}

HN heap_pop(MinHeap *h){
    HN out=h->a[0];
    h->pos[out.node]=-1;
    h->size--;
    if(h->size>0){
        h->a[0]=h->a[h->size];
        h->pos[h->a[0].node]=0;
        heap_down(h,0);
    }
    return out;
}
void heap_free(MinHeap *h){ free(h->a); free(h->pos); free(h); }

/* Dijkstra */
typedef struct { double *dist; int *prev; } DRes;
DRes dijkstra(Graph *g,int src,int dst){
    DRes res;
    res.dist=malloc(sizeof(double)*g->n_nodes);
    res.prev=malloc(sizeof(int)*g->n_nodes);
    for(size_t i=0;i<g->n_nodes;i++){ res.dist[i]=INFINITY; res.prev[i]=-1; }
    res.dist[src]=0.0;
    MinHeap *h=heap_create((int)g->n_nodes);
    heap_push_or_dec(h,src,0.0);
    while(h->size){
        HN hn=heap_pop(h);
        int u=hn.node; double du=hn.dist;
        if(du>res.dist[u]) continue;
        if(u==dst) break;
        for(Edge *e=g->adj[u]; e; e=e->next){
            int v=e->to;
            double nd=du+e->w;
            if(nd+1e-9<res.dist[v]){
                res.dist[v]=nd;
                res.prev[v]=u;
                heap_push_or_dec(h,v,nd);
            }
        }
    }
    heap_free(h);
    return res;
}

/* HTTP helpers */
void send_response(SOCKET fd,const char *status,const char *ctype,const char *body){
    char hdr[8192];
    int bodylen=(int)strlen(body);
    int n=snprintf(hdr,sizeof(hdr),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n"
        "\r\n",status,ctype,bodylen);
    send(fd,hdr,n,0);
    send(fd,body,bodylen,0);
}

char *build_nodes_geojson(Graph *g){
    size_t cap=g->n_nodes*200+1024;
    char *buf=malloc(cap);
    size_t p=0;
    p+=snprintf(buf+p,cap-p,"{\"type\":\"FeatureCollection\",\"features\":[");
    for(size_t i=0;i<g->n_nodes;i++){
        if(i) p+=snprintf(buf+p,cap-p,",");
        p+=snprintf(buf+p,cap-p,
            "{\"type\":\"Feature\",\"properties\":{\"id\":%lld},\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.8f,%.8f]}}",
            g->nodes[i].id,g->nodes[i].lon,g->nodes[i].lat);
    }
    p+=snprintf(buf+p,cap-p,"]}");
    return buf;
}

/* URL decode */
char *url_decode(const char *s){
    size_t n=strlen(s);
    char *out=malloc(n+1); char *d=out;
    for(const char *p=s;*p;p++){
        if(*p=='%' && isxdigit((unsigned char)p[1]) && isxdigit((unsigned char)p[2])){
            char hex[3]={p[1],p[2],0};
            *d++=(char)strtol(hex,NULL,16);
            p+=2;
        } else if(*p=='+') *d++=' ';
        else *d++=*p;
    }
    *d=0; return out;
}

/* Handle client */
void handle_client(SOCKET client_fd,Graph *g,char *nodes_geojson){
    char req[BUFSIZE];
    int r=recv(client_fd,req,sizeof(req)-1,0);
    if(r<=0){ closesocket(client_fd); return; }
    req[r]=0;
    char method[16], path[1024], proto[32];
    if(sscanf(req,"%15s %1023s %31s",method,path,proto)!=3){ closesocket(client_fd); return; }
    if(strcmp(method,"GET")!=0){ send_response(client_fd,"405 Method Not Allowed","text/plain","Method Not Allowed"); closesocket(client_fd); return; }

    if(strcmp(path,"/nodes")==0 || strncmp(path,"/nodes?",7)==0){
        send_response(client_fd,"200 OK","application/json",nodes_geojson);
        closesocket(client_fd); return;
    }

    if(strncmp(path,"/shortest",9)==0){
        char *q=strchr(path,'?');
        if(!q){ send_response(client_fd,"400 Bad Request","text/plain","Missing query"); closesocket(client_fd); return; }
        q++;
        char *qs=strdup(q);
        long long srcid=LLONG_MIN,dstid=LLONG_MIN;
        char *tok=strtok(qs,"&");
        while(tok){
            char *eq=strchr(tok,'=');
            if(eq){ *eq=0; char *k=tok; char *v=eq+1;
                if(strcmp(k,"src")==0) srcid=atoll(v);
                else if(strcmp(k,"dst")==0) dstid=atoll(v);
            }
            tok=strtok(NULL,"&");
        }
        free(qs);
        if(srcid==LLONG_MIN || dstid==LLONG_MIN){ send_response(client_fd,"400 Bad Request","text/plain","Missing src/dst"); closesocket(client_fd); return; }
        size_t sidx=find_index(g,srcid);
        size_t didx=find_index(g,dstid);
        if(sidx==(size_t)-1 || didx==(size_t)-1){ send_response(client_fd,"404 Not Found","text/plain","src/dst not found"); closesocket(client_fd); return; }
        DRes dr=dijkstra(g,(int)sidx,(int)didx);
        if(isinf(dr.dist[didx])){
            free(dr.dist); free(dr.prev);
            send_response(client_fd,"200 OK","application/json","{\"path\":[],\"distance\":null}");
            closesocket(client_fd); return;
        }

        int cur=(int)didx,cap=128,*stack=malloc(sizeof(int)*cap),sp=0;
        while(cur!=-1){
            if(sp>=cap){ cap*=2; stack=realloc(stack,sizeof(int)*cap); }
            stack[sp++]=cur;
            cur=dr.prev[cur];
        }

        size_t bufsz=sp*200+256;
        char *out=malloc(bufsz);
        size_t p=0;
        p+=snprintf(out+p,bufsz-p,"{\"path\":[");
        for(int i=sp-1;i>=0;i--){
            int idx=stack[i];
            if(i!=sp-1) p+=snprintf(out+p,bufsz-p,",");
            p+=snprintf(out+p,bufsz-p,"{\"id\":%lld,\"lat\":%.8f,\"lon\":%.8f}",
                        g->nodes[idx].id,g->nodes[idx].lat,g->nodes[idx].lon);
        }
        p+=snprintf(out+p,bufsz-p,"],\"distance\":%.3f}",dr.dist[didx]);
        send_response(client_fd,"200 OK","application/json",out);

        free(out); free(stack); free(dr.dist); free(dr.prev);
        closesocket(client_fd); return;
    }

    const char *msg="Shortest path server. /nodes and /shortest?src=<id>&dst=<id>";
    send_response(client_fd,"200 OK","text/plain",msg);
    closesocket(client_fd);
}

/* Main */
int main(int argc,char **argv){
    if(argc<3){ fprintf(stderr,"Usage: %s nodes.csv edges.csv\n",argv[0]); return 1; }
    printf("Loading graph...\n");
    Graph *g=load_graph(argv[1],argv[2]);
    if(!g){ fprintf(stderr,"Failed to load graph\n"); return 1; }
    printf("Loaded %zu nodes\n",g->n_nodes);
    char *nodes_geojson=build_nodes_geojson(g);

    WSADATA wsa;
    if(WSAStartup(MAKEWORD(2,2),&wsa)!=0){ fprintf(stderr,"WSAStartup failed\n"); return 1; }

    SOCKET server_fd=socket(AF_INET,SOCK_STREAM,0);
    if(server_fd==INVALID_SOCKET){ perror("socket"); return 1; }
    int opt=1; setsockopt(server_fd,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));

    struct sockaddr_in addr;
    addr.sin_family=AF_INET;
    addr.sin_addr.s_addr=INADDR_ANY;
    addr.sin_port=htons(PORT);

    if(bind(server_fd,(struct sockaddr*)&addr,sizeof(addr))==SOCKET_ERROR){ perror("bind"); return 1; }
    if(listen(server_fd,10)==SOCKET_ERROR){ perror("listen"); return 1; }
    printf("Server listening on port %d\n",PORT);

    while(1){
        struct sockaddr_in cli; int cli_len=sizeof(cli);
        SOCKET client_fd=accept(server_fd,(struct sockaddr*)&cli,&cli_len);
        if(client_fd==INVALID_SOCKET){ perror("accept"); continue; }
        handle_client(client_fd,g,nodes_geojson);
    }

    closesocket(server_fd);
    WSACleanup();
    return 0;
}
