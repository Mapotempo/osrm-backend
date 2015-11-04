/*

Copyright (c) 2015, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "../data_structures/static_rtree.hpp"
#include "../data_structures/edge_based_node.hpp"
#include "../include/osrm/coordinate.hpp"


/**
 * Matches coordinates against the OSM graph, and returns the closest
 * segment (node->node).
 *
 * The purpose of this tool is to perform bulk matching of GPS probe
 * points against the OSM graph.  If you have a "speed" value to go
 * along with each GPS probe point, you can use this to identify
 * all the travel speeds on a particular segment.  This can be used
 * to build a real-world speed profile for each segment.  osrm-prepare
 * can then be fed this data to weight the road speeds using
 * real data instead of profile speeds.
 * */
void load_node_information(
        const boost::filesystem::path &nodes_file,
        std::vector<FixedPointCoordinate> &m_coordinate_list,
        std::vector<NodeID> &m_nodeid_list)
{
    boost::filesystem::ifstream nodes_input_stream(nodes_file, std::ios::binary);

    QueryNode current_node;
    unsigned number_of_coordinates = 0;
    nodes_input_stream.read((char *)&number_of_coordinates, sizeof(unsigned));
    m_coordinate_list.resize(number_of_coordinates);
    m_nodeid_list.resize(number_of_coordinates);
    for (unsigned i = 0; i < number_of_coordinates; ++i)
    {
        nodes_input_stream.read((char *)&current_node, sizeof(QueryNode));
        m_coordinate_list.at(i) = FixedPointCoordinate(current_node.lat, current_node.lon);
        m_nodeid_list.at(i) = current_node.node_id;

        BOOST_ASSERT((std::abs(m_coordinate_list.at(i).lat) >> 30) == 0);
        BOOST_ASSERT((std::abs(m_coordinate_list.at(i).lon) >> 30) == 0);
    }
    nodes_input_stream.close();
}

int main(int argc, char *argv[])
{

    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <x.osrm>" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<FixedPointCoordinate> m_coordinate_list;
    std::vector<NodeID> m_osm_nodeids;

    std::string filename_base(argv[1]);

    load_node_information(filename_base + ".nodes", m_coordinate_list, m_osm_nodeids);

    std::shared_ptr<std::vector<FixedPointCoordinate>> coords = std::make_shared<std::vector<FixedPointCoordinate>>(m_coordinate_list);

    StaticRTree<EdgeBasedNode> rtree(filename_base+".ramIndex",filename_base+".fileIndex", coords);

    std::vector<std::pair<NodeID,NodeID>> results;
    rtree.IncrementalFindSegmentForCoordinateWithDistance(
            FixedPointCoordinate{static_cast<int>(0.0008368492126141084*COORDINATE_PRECISION),
                                 static_cast<int>(-0.0010878677368164062*COORDINATE_PRECISION)},
            results,
            10,
            90, // heading
            10); // heading range

    for (auto & r : results) {
        std::cout << m_osm_nodeids[r.first] << "->" << m_osm_nodeids[r.second] << std::endl;
    }

}
