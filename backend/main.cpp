// Map Navigator - C++17 backend (no external dependencies, POSIX sockets).
// REST API:
//   GET /api/health
//   GET /api/places
//   GET /api/graph
//   GET /api/route?from=<id>&to=<id>&algo=dijkstra|astar&mode=distance|time
// Also serves the frontend (frontend/index.html) at "/".

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------- Graph data
struct Node {
    int id;
    std::string name;
    double lat, lon;
};
struct Edge {
    int to;
    double km;      // road distance
    double speed;   // km/h
};

static const double ROAD_FACTOR = 1.3;  // straight-line -> approx road distance
static const double MAX_SPEED = 60.0;   // used for the A* time heuristic

static std::vector<Node> nodes = {
    {0, "India Gate", 28.6129, 77.2295},
    {1, "Connaught Place", 28.6315, 77.2167},
    {2, "Red Fort", 28.6562, 77.2410},
    {3, "Jama Masjid", 28.6507, 77.2334},
    {4, "New Delhi Railway Station", 28.6428, 77.2197},
    {5, "Old Delhi Railway Station", 28.6600, 77.2282},
    {6, "Humayun's Tomb", 28.5933, 77.2507},
    {7, "Lotus Temple", 28.5535, 77.2588},
    {8, "Qutub Minar", 28.5245, 77.1855},
    {9, "Akshardham Temple", 28.6127, 77.2773},
    {10, "Hauz Khas", 28.5494, 77.2001},
    {11, "IGI Airport", 28.5562, 77.1000},
    {12, "Saket", 28.5286, 77.2190},
    {13, "Karol Bagh", 28.6519, 77.1909},
    {14, "Lajpat Nagar", 28.5677, 77.2437},
    {15, "Nizamuddin Station", 28.5880, 77.2540},
    {16, "Lodhi Garden", 28.5931, 77.2197},
};

static std::vector<std::vector<Edge>> adj;
struct RawEdge { int a, b; double speed; };
static std::vector<RawEdge> rawEdges = {
    {0, 1, 40},  {0, 16, 40}, {0, 6, 40},  {0, 9, 50},  {1, 4, 30},  {1, 13, 35},
    {1, 3, 30},  {4, 5, 30},  {5, 2, 30},  {2, 3, 25},  {3, 4, 30},  {16, 6, 40},
    {16, 10, 40},{6, 15, 40}, {15, 14, 40},{14, 7, 40}, {7, 12, 40}, {12, 8, 45},
    {10, 8, 45}, {10, 11, 60},{11, 8, 60}, {13, 11, 60},{12, 10, 40},{9, 15, 50},
    {14, 16, 40},{5, 13, 35},
};

static double haversine(const Node& a, const Node& b) {
    const double R = 6371.0, d2r = M_PI / 180.0;
    double dlat = (b.lat - a.lat) * d2r, dlon = (b.lon - a.lon) * d2r;
    double h = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(a.lat * d2r) * std::cos(b.lat * d2r) * std::sin(dlon / 2) * std::sin(dlon / 2);
    return 2 * R * std::asin(std::sqrt(h));
}

static void buildGraph() {
    adj.assign(nodes.size(), {});
    for (auto& e : rawEdges) {
        double km = haversine(nodes[e.a], nodes[e.b]) * ROAD_FACTOR;
        adj[e.a].push_back({e.b, km, e.speed});
        adj[e.b].push_back({e.a, km, e.speed});
    }
}

// ------------------------------------------------------------ Route finding
struct RouteResult {
    bool found = false;
    std::vector<int> path;
    double km = 0, minutes = 0;
    int explored = 0;
    double ms = 0;
};

// cost of an edge under the chosen mode (distance in km, or time in hours)
static inline double edgeCost(const Edge& e, bool byTime) { return byTime ? e.km / e.speed : e.km; }

static RouteResult findRoute(int src, int dst, bool useAstar, bool byTime) {
    auto t0 = std::chrono::steady_clock::now();
    const double INF = 1e18;
    int n = (int)nodes.size();
    std::vector<double> g(n, INF);
    std::vector<int> prev(n, -1);
    std::vector<char> closed(n, 0);
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    auto h = [&](int v) -> double {
        if (!useAstar) return 0.0;
        double d = haversine(nodes[v], nodes[dst]);  // admissible: roads are >= straight line
        return byTime ? d / MAX_SPEED : d;
    };

    RouteResult r;
    g[src] = 0;
    pq.push({h(src), src});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (closed[u]) continue;
        closed[u] = 1;
        r.explored++;
        if (u == dst) break;
        for (auto& e : adj[u]) {
            double ng = g[u] + edgeCost(e, byTime);
            if (ng < g[e.to]) {
                g[e.to] = ng;
                prev[e.to] = u;
                pq.push({ng + h(e.to), e.to});
            }
        }
    }
    if (g[dst] < INF) {
        r.found = true;
        for (int v = dst; v != -1; v = prev[v]) r.path.push_back(v);
        std::reverse(r.path.begin(), r.path.end());
        for (size_t i = 0; i + 1 < r.path.size(); i++)
            for (auto& e : adj[r.path[i]])
                if (e.to == r.path[i + 1]) {
                    r.km += e.km;
                    r.minutes += e.km / e.speed * 60.0;
                    break;
                }
    }
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

// ---------------------------------------------------------------- JSON utils
static std::string jstr(const std::string& s) {
    std::string o = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else o += c;
    }
    return o + "\"";
}
static std::string num(double v, int prec = 4) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}
static std::string nodeJson(const Node& n) {
    return "{\"id\":" + std::to_string(n.id) + ",\"name\":" + jstr(n.name) +
           ",\"lat\":" + num(n.lat, 6) + ",\"lon\":" + num(n.lon, 6) + "}";
}
static std::string errJson(const std::string& m) { return "{\"error\":" + jstr(m) + "}"; }

// ------------------------------------------------------------------- HTTP
struct Response {
    int status = 200;
    std::string type = "application/json";
    std::string body;
};

static std::string urlDecode(const std::string& s) {
    std::string o;
    for (size_t i = 0; i < s.size(); i++) {
        if (s[i] == '%' && i + 2 < s.size()) {
            o += (char)std::strtol(s.substr(i + 1, 2).c_str(), nullptr, 16);
            i += 2;
        } else if (s[i] == '+') o += ' ';
        else o += s[i];
    }
    return o;
}
static std::map<std::string, std::string> parseQuery(const std::string& q) {
    std::map<std::string, std::string> m;
    std::stringstream ss(q);
    std::string kv;
    while (std::getline(ss, kv, '&')) {
        auto p = kv.find('=');
        if (p == std::string::npos) m[urlDecode(kv)] = "";
        else m[urlDecode(kv.substr(0, p))] = urlDecode(kv.substr(p + 1));
    }
    return m;
}
static bool parseId(const std::string& s, int& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    long v = std::strtol(s.c_str(), &end, 10);
    if (*end != '\0' || v < 0 || v >= (long)nodes.size()) return false;
    out = (int)v;
    return true;
}

static std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return "";
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static Response handle(const std::string& method, const std::string& path,
                       const std::map<std::string, std::string>& q) {
    Response r;
    if (method != "GET") { r.status = 405; r.body = errJson("method not allowed"); return r; }

    if (path == "/" || path == "/index.html") {
        const char* dir = std::getenv("FRONTEND_DIR");
        std::string html = readFile(std::string(dir ? dir : "frontend") + "/index.html");
        if (html.empty()) { r.status = 404; r.body = errJson("frontend not found"); return r; }
        r.type = "text/html; charset=utf-8";
        r.body = html;
        return r;
    }
    if (path == "/api/health") {
        r.body = "{\"status\":\"ok\",\"nodes\":" + std::to_string(nodes.size()) +
                 ",\"edges\":" + std::to_string(rawEdges.size()) + "}";
        return r;
    }
    if (path == "/api/places") {
        r.body = "{\"places\":[";
        for (size_t i = 0; i < nodes.size(); i++) r.body += (i ? "," : "") + nodeJson(nodes[i]);
        r.body += "]}";
        return r;
    }
    if (path == "/api/graph") {
        r.body = "{\"nodes\":[";
        for (size_t i = 0; i < nodes.size(); i++) r.body += (i ? "," : "") + nodeJson(nodes[i]);
        r.body += "],\"edges\":[";
        for (size_t i = 0; i < rawEdges.size(); i++) {
            auto& e = rawEdges[i];
            double km = haversine(nodes[e.a], nodes[e.b]) * ROAD_FACTOR;
            r.body += (i ? "," : "") + std::string("{\"a\":") + std::to_string(e.a) +
                      ",\"b\":" + std::to_string(e.b) + ",\"km\":" + num(km, 2) +
                      ",\"speed\":" + num(e.speed, 0) + "}";
        }
        r.body += "]}";
        return r;
    }
    if (path == "/api/route") {
        int from, to;
        if (!q.count("from") || !q.count("to") || !parseId(q.at("from"), from) || !parseId(q.at("to"), to)) {
            r.status = 400;
            r.body = errJson("'from' and 'to' must be valid place ids (see /api/places)");
            return r;
        }
        std::string algo = q.count("algo") ? q.at("algo") : "astar";
        std::string mode = q.count("mode") ? q.at("mode") : "distance";
        if ((algo != "astar" && algo != "dijkstra") || (mode != "distance" && mode != "time")) {
            r.status = 400;
            r.body = errJson("algo must be dijkstra|astar, mode must be distance|time");
            return r;
        }
        RouteResult rr = findRoute(from, to, algo == "astar", mode == "time");
        if (!rr.found) { r.status = 404; r.body = errJson("no route found"); return r; }
        std::ostringstream os;
        os << "{\"from\":" << jstr(nodes[from].name) << ",\"to\":" << jstr(nodes[to].name)
           << ",\"algo\":" << jstr(algo) << ",\"mode\":" << jstr(mode)
           << ",\"distance_km\":" << num(rr.km, 2) << ",\"duration_min\":" << num(rr.minutes, 1)
           << ",\"nodes_explored\":" << rr.explored << ",\"compute_ms\":" << num(rr.ms, 4)
           << ",\"path\":[";
        for (size_t i = 0; i < rr.path.size(); i++) os << (i ? "," : "") << nodeJson(nodes[rr.path[i]]);
        os << "]}";
        r.body = os.str();
        return r;
    }
    r.status = 404;
    r.body = errJson("not found");
    return r;
}

static const char* statusText(int s) {
    switch (s) {
        case 200: return "OK";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        default: return "Error";
    }
}

static void serveClient(int fd) {
    timeval tv{5, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    std::string req;
    char buf[2048];
    while (req.find("\r\n\r\n") == std::string::npos && req.size() < 16384) {
        ssize_t n = recv(fd, buf, sizeof buf, 0);
        if (n <= 0) break;
        req.append(buf, n);
    }
    std::istringstream is(req);
    std::string method, target, version;
    is >> method >> target >> version;

    Response r;
    if (method.empty() || target.empty()) {
        r.status = 400;
        r.body = errJson("bad request");
    } else if (method == "OPTIONS") {
        r.status = 200;  // CORS preflight
    } else {
        std::string path = target, query;
        auto qp = target.find('?');
        if (qp != std::string::npos) { path = target.substr(0, qp); query = target.substr(qp + 1); }
        r = handle(method, path, parseQuery(query));
    }
    std::ostringstream out;
    out << "HTTP/1.1 " << r.status << " " << statusText(r.status) << "\r\n"
        << "Content-Type: " << r.type << "\r\n"
        << "Content-Length: " << r.body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
        << "Access-Control-Allow-Headers: Content-Type\r\n"
        << "Connection: close\r\n\r\n"
        << r.body;
    std::string s = out.str();
    size_t sent = 0;
    while (sent < s.size()) {
        ssize_t n = send(fd, s.data() + sent, s.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) break;
        sent += n;
    }
    close(fd);
}

int main() {
    signal(SIGPIPE, SIG_IGN);
    buildGraph();
    const char* p = std::getenv("PORT");
    int port = p ? std::atoi(p) : 8080;

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof opt);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(srv, (sockaddr*)&addr, sizeof addr) < 0 || listen(srv, 64) < 0) {
        perror("bind/listen");
        return 1;
    }
    std::cout << "Map Navigator listening on http://0.0.0.0:" << port << std::endl;
    while (true) {
        int fd = accept(srv, nullptr, nullptr);
        if (fd < 0) continue;
        std::thread(serveClient, fd).detach();
    }
}
