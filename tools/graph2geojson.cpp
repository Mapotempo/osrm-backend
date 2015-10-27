#include "../data_structures/static_graph.hpp"
#include "../server/data_structures/internal_datafacade.hpp"
#include "../data_structures/query_edge.hpp"
#include "../data_structures/original_edge_data.hpp"
#include "../util/simple_logger.hpp"
#include "../util/graph_loader.hpp"
#include "../util/routed_options.hpp"
#include "../algorithms/coordinate_calculation.hpp"

#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

#include <unordered_set>

using EdgeData = QueryEdge::EdgeData;
using QueryGraph = StaticGraph<EdgeData>;
using FacadeT = InternalDataFacade<QueryEdge::EdgeData>;

struct VisualizationData {
    std::vector<NodeID> geometry_ids;
    std::vector<std::size_t> geometry_offsets;
    std::vector<EdgeWeight> durations;
};

VisualizationData getVisualizationData(FacadeT& facade)
{
    VisualizationData data;

    VisualizationData vis_data;

    std::unordered_set<unsigned> covered_edge_ids;

    for (const auto first : osrm::irange<std::size_t>(0, facade.GetNumberOfNodes())) {
        std::vector<NodeID> parent(facade.GetNumberOfNodes(), SPECIAL_NODEID);
        // we safe the last node of the geometry to re-construct the compressed edges
        std::vector<NodeID> last_node(facade.GetNumberOfNodes(), SPECIAL_NODEID);

        std::deque<NodeID> queue;
        queue.push_back(first);
        parent[first] = first;
        while (queue.size() > 0)
        {
            auto source = queue.front();
            queue.pop_front();

            for (auto edge : facade.GetAdjacentEdgeRange(source))
            {
                const auto& data = facade.GetEdgeData(edge);
                if (data.shortcut) continue;
                auto edge_id = data.id;
                auto target = facade.GetTarget(edge);

                if (covered_edge_ids.find(edge_id) != covered_edge_ids.end())
                {
                    continue;
                }
                covered_edge_ids.insert(edge_id);

                if (parent[target] == SPECIAL_NODEID)
                {
                    queue.push_back(target);
                    parent[target] = source;

                    auto via_node_id = facade.GetGeometryIndexForEdgeID(edge_id);

                    if (facade.EdgeIsCompressed(edge_id))
                    {
                        std::vector<NodeID> uncompressed_geometry;
                        facade.GetUncompressedGeometry(via_node_id, uncompressed_geometry);
                        if (data.forward) {
                            last_node[target] = uncompressed_geometry.back();
                        } else {
                            last_node[target] = uncompressed_geometry.front();
                        }

                        if (last_node[source] != SPECIAL_NODEID)
                        {
                            vis_data.geometry_offsets.push_back(vis_data.geometry_ids.size());
                            vis_data.durations.push_back(data.distance);
                            if (data.forward) {
                                vis_data.geometry_ids.push_back(last_node[source]);
                                vis_data.geometry_ids.insert(vis_data.geometry_ids.end(),
                                        uncompressed_geometry.begin(),
                                        uncompressed_geometry.end());
                            } else {
                                vis_data.geometry_ids.insert(vis_data.geometry_ids.end(),
                                        uncompressed_geometry.begin(),
                                        uncompressed_geometry.end());
                                vis_data.geometry_ids.push_back(last_node[source]);
                            }
                        }
                    }
                    else
                    {
                        last_node[target] = via_node_id;
                        if (last_node[source] != SPECIAL_NODEID)
                        {
                            vis_data.geometry_offsets.push_back(vis_data.geometry_ids.size());
                            vis_data.durations.push_back(data.distance);
                            if (data.forward) {
                                vis_data.geometry_ids.push_back(last_node[source]);
                                vis_data.geometry_ids.push_back(via_node_id);
                            } else {
                                vis_data.geometry_ids.push_back(via_node_id);
                                vis_data.geometry_ids.push_back(last_node[source]);
                            }
                        }
                    }
                }
            }
        }
    }

    // add senetile
    vis_data.geometry_offsets.push_back(vis_data.geometry_ids.size());

    return vis_data;
}

// Taken from http://stackoverflow.com/questions/2353211/hsl-to-rgb-color-conversion
std::tuple<double, double, double> hslToRgb(double h, double s, double l)
{
    double r, g, b;

    if(s == 0)
    {
        r = g = b = l; // achromatic
    }
    else
    {
        auto hue2rgb = [](double p, double q, double t){
            if(t < 0.) t += 1.;
            if(t > 1.) t -= 1.;
            if(t < 1./6.) return p + (q - p) * 6. * t;
            if(t < 1./2.) return q;
            if(t < 2./3.) return p + (q - p) * (2./3. - t) * 6.;
            return p;
        };

        double q = l < 0.5 ? l * (1. + s) : l + s - l * s;
        double p = 2. * l - q;
        r = hue2rgb(p, q, h + 1./3.);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1./3.);
    }

    return std::make_tuple(r, g, b);
}

void writeData(std::ofstream& out, FacadeT& facade, const VisualizationData& data)
{
    std::vector<FixedPointCoordinate> geometry(data.geometry_ids.size());
    std::transform(data.geometry_ids.begin(), data.geometry_ids.end(), geometry.begin(),
            [&facade](NodeID id) {
                return facade.GetCoordinateOfNode(id);
            });

    // reconstruct speed values
    std::vector<double> speeds;
    for (const auto id : osrm::irange<NodeID>(0, data.geometry_offsets.size() - 1))
    {
        auto begin = data.geometry_offsets[id];
        auto end = data.geometry_offsets[id+1];
        auto length = 0.0;
        for (auto current = begin + 1; current < end; ++current)
        {
            length += coordinate_calculation::euclidean_distance(geometry[current-1], geometry[current]);
        }
        auto speed = std::numeric_limits<double>::max();
        if (data.durations[id] > 0)
        {
            // speed in m/s
            speed = length / (data.durations[id] / 10.0);
        }
        speeds.push_back(speed);
    }

    double max_speed = 50; // m/s = 180 km/h

    out << "{\"type\": \"FeatureCollection\", \"features\": [";
    for (const auto id : osrm::irange<NodeID>(0, data.geometry_offsets.size() - 1))
    {
        auto begin = data.geometry_offsets[id];
        auto end = data.geometry_offsets[id+1];
        double r, g, b;
        double hue = speeds[id] / max_speed;
        if (hue > 1) hue = 1.0;
        std::tie(r, g, b) = hslToRgb(hue, 1.0, 0.5);
        auto color_code = boost::format("#%02X%02X%02X") % (int)static_cast<unsigned char>(255*r) %
                                                       (int)static_cast<unsigned char>(255*g) %
                                                       (int)static_cast<unsigned char>(255*b);
        out << "{\"type\": \"Feature\", \"properties\": { \"color\": \"" << color_code <<
               "\", \"speed\": " << speeds[id] << "}, " <<
               "\"geometry\": { \"type\": \"LineString\", \"coordinates\": [";
        for (const auto idx : osrm::irange<std::size_t>(begin, end))
        {
            out << "[" << (geometry[idx].lon / COORDINATE_PRECISION) << ","
                << (geometry[idx].lat / COORDINATE_PRECISION) << "],";
        }
        // HACK to remove the last ','
        out.seekp(-1, out.cur);
        out << "]}},\n";
    }
    // HACK to remove the last ','
    out.seekp(-2, out.cur);
    out << "]}" << std::endl;;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        SimpleLogger().Write(logWARNING) << "usage:\n" << argv[0] << " <.osrm> <output.geojson>";
        return -1;
    }
    boost::filesystem::path osrm_path(argv[1]);
    ServerPaths paths;
    paths["base"] = osrm_path;
    populate_base_path(paths);
    FacadeT facade(paths);

    VisualizationData data = getVisualizationData(facade);

    std::ofstream out(argv[2]);
    writeData(out, facade, data);
}
