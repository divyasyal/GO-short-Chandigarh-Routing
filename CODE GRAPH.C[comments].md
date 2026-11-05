/\* graph\_server.c

&nbsp;\* Minimal shortest-path HTTP server for demo.

&nbsp;\* Compile (MSYS2): gcc -O2 graph\_server.c -o graph\_server -lm -lws2\_32

&nbsp;\*

&nbsp;\* Usage: ./graph\_server nodes.csv edges.csv

&nbsp;\*

&nbsp;\* Endpoints:

&nbsp;\*   GET /nodes

&nbsp;\*     -> returns GeoJSON FeatureCollection of nodes

&nbsp;\*   GET /shortest?src=<nodeid>\&dst=<nodeid>

&nbsp;\*     -> returns JSON { path: \[ {id,lat,lon}, ... ], distance: meters }

&nbsp;\*

&nbsp;\* Note: nodes.csv format: id,lat,lon

&nbsp;\*       edges.csv format: from,to,weight,wayid

&nbsp;\*

&nbsp;\* CORS header included for browser access.

&nbsp;\*/



\#define \_GNU\_SOURCE                // Enable GNU extensions for some functions

\#include <stdio.h>                 // Standard I/O

\#include <stdlib.h>                // Standard library (malloc, free)

\#include <string.h>                // String manipulation

\#include <ctype.h>                 // Character functions (isdigit, isxdigit)

\#include <math.h>                  // Math functions (sin, cos, atan2)

\#include <limits.h>                // For LLONG\_MIN

\#include <stdbool.h>               // Boolean type

\#include <winsock2.h>              // Windows Sockets API

\#include <ws2tcpip.h>              // For inet\_pton, etc.

\#pragma comment(lib, "ws2\_32.lib") // Link Winsock library



\#define PORT 8080                  // Server port

\#define BUFSIZE 65536              // Buffer size for reading requests



/\* Node structure: holds id and coordinates \*/

typedef struct {

&nbsp;   long long id;                 // Unique node ID

&nbsp;   double lat, lon;              // Latitude and longitude

} Node;



/\* Edge structure: adjacency list node \*/

typedef struct Edge {

&nbsp;   int to;                       // Index of target node

&nbsp;   double w;                      // Weight (distance)

&nbsp;   struct Edge \*next;             // Next edge in linked list

} Edge;



/\* Graph structure \*/

typedef struct {

&nbsp;   Node \*nodes;                  // Array of nodes

&nbsp;   size\_t n\_nodes;               // Number of nodes

&nbsp;   Edge \*\*adj;                   // Array of adjacency lists

} Graph;



/\* Convert degrees to radians \*/

static inline double deg2rad(double d){ 

&nbsp;   return d \* M\_PI / 180.0; 

}



/\* Haversine distance between two coordinates (meters) \*/

double haversine(double lat1,double lon1,double lat2,double lon2){

&nbsp;   double R = 6371000.0;                               // Earth radius in meters

&nbsp;   double dlat = deg2rad(lat2-lat1);                   // Latitude difference

&nbsp;   double dlon = deg2rad(lon2-lon1);                   // Longitude difference

&nbsp;   double a = sin(dlat/2.0)\*sin(dlat/2.0) + 

&nbsp;              cos(deg2rad(lat1))\*cos(deg2rad(lat2))\*

&nbsp;              sin(dlon/2.0)\*sin(dlon/2.0);            // Haversine formula

&nbsp;   double c = 2.0 \* atan2(sqrt(a), sqrt(1.0 - a));    // Angular distance

&nbsp;   return R \* c;                                       // Distance in meters

}



/\* Map node ID to its index in nodes array (linear search) \*/

size\_t find\_index(Graph \*g, long long id){

&nbsp;   for(size\_t i=0;i<g->n\_nodes;i++) 

&nbsp;       if(g->nodes\[i].id==id) return i;              // Found, return index

&nbsp;   return (size\_t)-1;                                // Not found

}



/\* Load nodes and edges from CSV files into Graph \*/

Graph \*load\_graph(const char \*nodes\_csv,const char \*edges\_csv){

&nbsp;   FILE \*f=fopen(nodes\_csv,"r");                     // Open nodes CSV

&nbsp;   if(!f){ perror("nodes.csv"); return NULL; }       // Error handling



&nbsp;   char line\[4096];                                   // Line buffer

&nbsp;   size\_t count=0;



&nbsp;   /\* Count lines with commas to estimate number of nodes \*/

&nbsp;   while(fgets(line,sizeof(line),f)) 

&nbsp;       if(strchr(line,',')) count++;

&nbsp;   rewind(f);                                        // Reset file pointer



&nbsp;   Node \*nodes=calloc(count,sizeof(Node));           // Allocate nodes array

&nbsp;   size\_t idx=0;



&nbsp;   /\* Read nodes from CSV \*/

&nbsp;   while(fgets(line,sizeof(line),f)){

&nbsp;       if(strchr(line,',')==NULL) continue;          // Skip invalid lines

&nbsp;       long long id; double lat, lon;

&nbsp;       if(sscanf(line,"%lld,%lf,%lf",\&id,\&lat,\&lon)>=3){ // Parse id, lat, lon

&nbsp;           nodes\[idx].id=id;

&nbsp;           nodes\[idx].lat=lat;

&nbsp;           nodes\[idx].lon=lon;

&nbsp;           idx++;

&nbsp;       }

&nbsp;   }

&nbsp;   fclose(f);



&nbsp;   Graph \*g=calloc(1,sizeof(Graph));                 // Allocate graph

&nbsp;   g->nodes=nodes;

&nbsp;   g->n\_nodes=idx;

&nbsp;   g->adj=calloc(g->n\_nodes,sizeof(Edge\*));         // Allocate adjacency lists



&nbsp;   /\* Read edges CSV \*/

&nbsp;   f=fopen(edges\_csv,"r");

&nbsp;   if(!f){ perror("edges.csv"); return NULL; }

&nbsp;   while(fgets(line,sizeof(line),f)){

&nbsp;       if(strchr(line,',')==NULL) continue;

&nbsp;       long long fromid,toid; double w; int wayid=0;

&nbsp;       if(sscanf(line,"%lld,%lld,%lf,%d",\&fromid,\&toid,\&w,\&wayid)<3) continue;



&nbsp;       size\_t fi=find\_index(g,fromid);               // From index

&nbsp;       size\_t ti=find\_index(g,toid);                 // To index

&nbsp;       if(fi==(size\_t)-1 || ti==(size\_t)-1) continue;



&nbsp;       Edge \*e=malloc(sizeof(Edge));                 // Create edge

&nbsp;       e->to=(int)ti;

&nbsp;       e->w=w;

&nbsp;       e->next=g->adj\[fi];                           // Insert at head

&nbsp;       g->adj\[fi]=e;

&nbsp;   }

&nbsp;   fclose(f);

&nbsp;   return g;                                         // Return graph

}



/\* ===================== Min-heap for Dijkstra ===================== \*/

typedef struct { int node; double dist; } HN;       // Heap node

typedef struct {

&nbsp;   HN \*a; int size; int cap; int \*pos;             // Heap array, size, capacity, positions

} MinHeap;



/\* Create min-heap of size n \*/

MinHeap \*heap\_create(int n){

&nbsp;   MinHeap \*h=malloc(sizeof(MinHeap));

&nbsp;   h->cap=n+5;

&nbsp;   h->a=malloc(sizeof(HN)\*(h->cap));

&nbsp;   h->size=0;

&nbsp;   h->pos=malloc(sizeof(int)\*n);

&nbsp;   for(int i=0;i<n;i++) h->pos\[i]=-1;             // Initialize positions

&nbsp;   return h;

}



/\* Swap two heap elements and update positions \*/

void heap\_swap(MinHeap \*h,int i,int j){

&nbsp;   HN t=h->a\[i]; h->a\[i]=h->a\[j]; h->a\[j]=t;

&nbsp;   h->pos\[h->a\[i].node]=i;

&nbsp;   h->pos\[h->a\[j].node]=j;

}



/\* Move heap element up to maintain min-heap \*/

void heap\_up(MinHeap \*h,int i){

&nbsp;   while(i>0){

&nbsp;       int p=(i-1)/2;                              // Parent index

&nbsp;       if(h->a\[p].dist<=h->a\[i].dist) break;       // Heap property ok

&nbsp;       heap\_swap(h,p,i); i=p;                      // Swap with parent

&nbsp;   }

}



/\* Move heap element down to maintain min-heap \*/

void heap\_down(MinHeap \*h,int i){

&nbsp;   int n=h->size;

&nbsp;   while(1){

&nbsp;       int l=2\*i+1,r=2\*i+2,s=i;                    // Left, right child

&nbsp;       if(l<n \&\& h->a\[l].dist<h->a\[s].dist) s=l;

&nbsp;       if(r<n \&\& h->a\[r].dist<h->a\[s].dist) s=r;

&nbsp;       if(s==i) break;

&nbsp;       heap\_swap(h,i,s); i=s;

&nbsp;   }

}



/\* Insert or decrease key in heap \*/

void heap\_push\_or\_dec(MinHeap \*h,int node,double d){

&nbsp;   if(h->pos\[node]!=-1){                            // Node already in heap

&nbsp;       int i=h->pos\[node];

&nbsp;       if(d<h->a\[i].dist){ h->a\[i].dist=d; heap\_up(h,i); }

&nbsp;   } else {

&nbsp;       int i=h->size++;

&nbsp;       h->a\[i].node=node;

&nbsp;       h->a\[i].dist=d;

&nbsp;       h->pos\[node]=i;

&nbsp;       heap\_up(h,i);

&nbsp;   }

}



/\* Pop the min element from heap \*/

HN heap\_pop(MinHeap \*h){

&nbsp;   HN out=h->a\[0];                                 // Minimum element

&nbsp;   h->pos\[out.node]=-1;

&nbsp;   h->size--;

&nbsp;   if(h->size>0){

&nbsp;       h->a\[0]=h->a\[h->size];                     // Move last element to root

&nbsp;       h->pos\[h->a\[0].node]=0;

&nbsp;       heap\_down(h,0);                             // Restore heap property

&nbsp;   }

&nbsp;   return out;

}



/\* Free heap memory \*/

void heap\_free(MinHeap \*h){ free(h->a); free(h->pos); free(h); }



/\* ===================== Dijkstra's algorithm ===================== \*/

typedef struct { double \*dist; int \*prev; } DRes;   // Stores distances and previous nodes



DRes dijkstra(Graph \*g,int src,int dst){

&nbsp;   DRes res;

&nbsp;   res.dist=malloc(sizeof(double)\*g->n\_nodes);      // Allocate distance array

&nbsp;   res.prev=malloc(sizeof(int)\*g->n\_nodes);        // Allocate previous node array



&nbsp;   for(size\_t i=0;i<g->n\_nodes;i++){               // Initialize arrays

&nbsp;       res.dist\[i]=INFINITY;

&nbsp;       res.prev\[i]=-1;

&nbsp;   }

&nbsp;   res.dist\[src]=0.0;                               // Distance to source = 0



&nbsp;   MinHeap \*h=heap\_create((int)g->n\_nodes);        // Create min-heap

&nbsp;   heap\_push\_or\_dec(h,src,0.0);                     // Push source



&nbsp;   while(h->size){

&nbsp;       HN hn=heap\_pop(h);                           // Pop node with min distance

&nbsp;       int u=hn.node; double du=hn.dist;



&nbsp;       if(du>res.dist\[u]) continue;                 // Skip outdated entry

&nbsp;       if(u==dst) break;                            // Stop if reached destination



&nbsp;       for(Edge \*e=g->adj\[u]; e; e=e->next){       // Visit neighbors

&nbsp;           int v=e->to;

&nbsp;           double nd=du+e->w;                       // Tentative distance

&nbsp;           if(nd+1e-9 < res.dist\[v]){               // If shorter path found

&nbsp;               res.dist\[v]=nd;

&nbsp;               res.prev\[v]=u;

&nbsp;               heap\_push\_or\_dec(h,v,nd);            // Update heap

&nbsp;           }

&nbsp;       }

&nbsp;   }

&nbsp;   heap\_free(h);                                    // Free heap

&nbsp;   return res;                                      // Return distances and path info

}



/\* ===================== HTTP helpers ===================== \*/

/\* Send HTTP response \*/

void send\_response(SOCKET fd,const char \*status,const char \*ctype,const char \*body){

&nbsp;   char hdr\[8192];

&nbsp;   int bodylen=(int)strlen(body);

&nbsp;   int n=snprintf(hdr,sizeof(hdr),

&nbsp;       "HTTP/1.1 %s\\r\\n"

&nbsp;       "Content-Type: %s\\r\\n"

&nbsp;       "Content-Length: %d\\r\\n"

&nbsp;       "Access-Control-Allow-Origin: \*\\r\\n"

&nbsp;       "Connection: close\\r\\n"

&nbsp;       "\\r\\n",status,ctype,bodylen);

&nbsp;   send(fd,hdr,n,0);                                // Send header

&nbsp;   send(fd,body,bodylen,0);                         // Send body

}



/\* Build GeoJSON string for all nodes \*/

char \*build\_nodes\_geojson(Graph \*g){

&nbsp;   size\_t cap=g->n\_nodes\*200+1024;                  // Approx buffer size

&nbsp;   char \*buf=malloc(cap);

&nbsp;   size\_t p=0;

&nbsp;   p+=snprintf(buf+p,cap-p,"{\\"type\\":\\"FeatureCollection\\",\\"features\\":\[");

&nbsp;   for(size\_t i=0;i<g->n\_nodes;i++){

&nbsp;       if(i) p+=snprintf(buf+p,cap-p,",");

&nbsp;       p+=snprintf(buf+p,cap-p,

&nbsp;           "{\\"type\\":\\"Feature\\",\\"properties\\":{\\"id\\":%lld},\\"geometry\\":{\\"type\\":\\"Point\\",\\"coordinates\\":\[%.8f,%.8f]}}",

&nbsp;           g->nodes\[i].id,g->nodes\[i].lon,g->nodes\[i].lat);

&nbsp;   }

&nbsp;   p+=snprintf(buf+p,cap-p,"]}");

&nbsp;   return buf;

}



/\* Decode URL-encoded strings \*/

char \*url\_decode(const char \*s){

&nbsp;   size\_t n=strlen(s);

&nbsp;   char \*out=malloc(n+1); char \*d=out;

&nbsp;   for(const char \*p=s;\*p;p++){

&nbsp;       if(\*p=='%' \&\& isxdigit((unsigned char)p\[1]) \&\& isxdigit((unsigned char)p\[2])){

&nbsp;           char hex\[3]={p\[1],p\[2],0};

&nbsp;           \*d++=(char)strtol(hex,NULL,16);

&nbsp;           p+=2;

&nbsp;       } else if(\*p=='+') \*d++=' ';

&nbsp;       else \*d++=\*p;

&nbsp;   }

&nbsp;   \*d=0; return out;

}



/\* ===================== Handle client requests ===================== \*/

void handle\_client(SOCKET client\_fd,Graph \*g,char \*nodes\_geojson){

&nbsp;   char req\[BUFSIZE];

&nbsp;   int r=recv(client\_fd,req,sizeof(req)-1,0);        // Receive HTTP request

&nbsp;   if(r<=0){ closesocket(client\_fd); return; }     // Connection closed

&nbsp;   req\[r]=0;                                        // Null-terminate request



&nbsp;   char method\[16], path\[1024], proto\[32];

&nbsp;   if(sscanf(req,"%15s %1023s %31s",method,path,proto)!=3){ closesocket(client\_fd); return; }



&nbsp;   if(strcmp(method,"GET")!=0){                     // Only allow GET

&nbsp;       send\_response(client\_fd,"405 Method Not Allowed","text/plain","Method Not Allowed");

&nbsp;       closesocket(client\_fd); return;

&nbsp;   }



&nbsp;   /\* --- Handle /nodes --- \*/

&nbsp;   if(strcmp(path,"/nodes")==0 || strncmp(path,"/nodes?",7)==0){

&nbsp;       send\_response(client\_fd,"200 OK","application/json",nodes\_geojson);

&nbsp;       closesocket(client\_fd); return;

&nbsp;   }



&nbsp;   /\* --- Handle /shortest --- \*/

&nbsp;   if(strncmp(path,"/shortest",9)==0){

&nbsp;       char \*q=strchr(path,'?');                     // Find query string

&nbsp;       if(!q){ send\_response(client\_fd,"400 Bad Request","text/plain","Missing query"); closesocket(client\_fd); return; }

&nbsp;       q++;

&nbsp;       char \*qs=strdup(q);



&nbsp;       long long srcid=LLONG\_MIN,dstid=LLONG\_MIN;

&nbsp;       char \*tok=strtok(qs,"\&");

&nbsp;       while(tok){

&nbsp;           char \*eq=strchr(tok,'=');

&nbsp;           if(eq){ \*eq=0; char \*k=tok; char \*v=eq+1;

&nbsp;               if(strcmp(k,"src")==0) srcid=atoll(v);

&nbsp;               else if(strcmp(k,"dst")==0) dstid=atoll(v);

&nbsp;           }

&nbsp;           tok=strtok(NULL,"\&");

&nbsp;       }

&nbsp;       free(qs);



&nbsp;       if(srcid==LLONG\_MIN || dstid==LLONG\_MIN){ send\_response(client\_fd,"400 Bad Request","text/plain","Missing src/dst"); closesocket(client\_fd); return; }



&nbsp;       size\_t sidx=find\_index(g,srcid);

&nbsp;       size\_t didx=find\_index(g,dstid);

&nbsp;       if(sidx==(size\_t)-1 || didx==(size\_t)-1){ send\_response(client\_fd,"404 Not Found","text/plain","src/dst not found"); closesocket(client\_fd); return; }



&nbsp;       DRes dr=dijkstra(g,(int)sidx,(int)didx);    // Compute shortest path



&nbsp;       if(isinf(dr.dist\[didx])){                    // No path

&nbsp;           free(dr.dist); free(dr.prev);

&nbsp;           send\_response(client\_fd,"200 OK","application/json","{\\"path\\":\[],\\"distance\\":null}");

&nbsp;           closesocket(client\_fd); return;

&nbsp;       }



&nbsp;       /\* Build path from dst to src using prev array \*/

&nbsp;       int cur=(int)didx, cap=128, \*stack=malloc(sizeof(int)\*cap), sp=0;

&nbsp;       while(cur!=-1){

&nbsp;           if(sp>=cap){ cap\*=2; stack=realloc(stack,sizeof(int)\*cap); }

&nbsp;           stack\[sp++]=cur;

&nbsp;           cur=dr.prev\[cur];

&nbsp;       }



&nbsp;       /\* Build JSON response \*/

&nbsp;       size\_t bufsz=sp\*200+256;

&nbsp;       char \*out=malloc(bufsz);

&nbsp;       size\_t p=0;

&nbsp;       p+=snprintf(out+p,bufsz-p,"{\\"path\\":\[");

&nbsp;       for(int i=sp-1;i>=0;i--){

&nbsp;           int idx=stack\[i];

&nbsp;           if(i!=sp-1) p+=snprintf(out+p,bufsz-p,",");

&nbsp;           p+=snprintf(out+p,bufsz-p,"{\\"id\\":%lld,\\"lat\\":%.8f,\\"lon\\":%.8f}",

&nbsp;                       g->nodes\[idx].id,g->nodes\[idx].lat,g->nodes\[idx].lon);

&nbsp;       }

&nbsp;       p+=snprintf(out+p,bufsz-p,"],\\"distance\\":%.3f}",dr.dist\[didx]);

&nbsp;       send\_response(client\_fd,"200 OK","application/json",out);



&nbsp;       free(out); free(stack); free(dr.dist); free(dr.prev);

&nbsp;       closesocket(client\_fd); return;

&nbsp;   }



&nbsp;   /\* Default response \*/

&nbsp;   const char \*msg="Shortest path server. /nodes and /shortest?src=<id>\&dst=<id>";

&nbsp;   send\_response(client\_fd,"200 OK","text/plain",msg);

&nbsp;   closesocket(client\_fd);

}



/\* ===================== Main ===================== \*/

int main(int argc,char \*\*argv){

&nbsp;   if(argc<3){ fprintf(stderr,"Usage: %s nodes.csv edges.csv\\n",argv\[0]); return 1; }



&nbsp;   printf("Loading graph...\\n");

&nbsp;   Graph \*g=load\_graph(argv\[1],argv\[2]);              // Load nodes and edges

&nbsp;   if(!g){ fprintf(stderr,"Failed to load graph\\n"); return 1; }

&nbsp;   printf("Loaded %zu nodes\\n",g->n\_nodes);



&nbsp;   char \*nodes\_geojson=build\_nodes\_geojson(g);       // Pre-build nodes JSON



&nbsp;   WSADATA wsa;

&nbsp;   if(WSAStartup(MAKEWORD(2,2),\&wsa)!=0){ fprintf(stderr,"WSAStartup failed\\n"); return 1; }



&nbsp;   SOCKET server\_fd=socket(AF\_INET,SOCK\_STREAM,0);   // Create TCP socket

&nbsp;   if(server\_fd==INVALID\_SOCKET){ perror("socket"); return 1; }



&nbsp;   int opt=1; setsockopt(server\_fd,SOL\_SOCKET,SO\_REUSEADDR,(const char\*)\&opt,sizeof(opt));



&nbsp;   struct sockaddr\_in addr;

&nbsp;   addr.sin\_family=AF\_INET;

&nbsp;   addr.sin\_addr.s\_addr=INADDR\_ANY;                  // Bind to all interfaces

&nbsp;   addr.sin\_port=htons(PORT);                        // Port in network byte order



&nbsp;   if(bind(server\_fd,(struct sockaddr\*)\&addr,sizeof(addr))==SOCKET\_ERROR){ perror("bind"); return 1; }

&nbsp;   if(listen(server\_fd,10)==SOCKET\_ERROR){ perror("listen"); return 1; }

&nbsp;   printf("Server listening on port %d\\n",PORT);



&nbsp;   /\* Main server loop \*/

&nbsp;   while(1){

&nbsp;       struct sockaddr\_in cli; int cli\_len=sizeof(cli);

&nbsp;       SOCKET client\_fd=accept(server\_fd,(struct sockaddr\*)\&cli,\&cli\_len); // Accept connection

&nbsp;       if(client\_fd==INVALID\_SOCKET){ perror("accept"); continue; }

&nbsp;       handle\_client(client\_fd,g,nodes\_geojson);   // Handle request

&nbsp;   }



&nbsp;   closesocket(server\_fd);

&nbsp;   WSACleanup();

&nbsp;   return 0;

}



