#include <iostream>

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/pretty_print.hpp>

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    std::cout << "Creating graph with nodes indices:\n\n";
    std::cout << "  2 | 5 | 8\n"
              << "  --+---+--\n"
              << "  1 | 4 | 7\n"
              << "  --+---+--\n"
              << "  0 | 3 | 6\n\n";

    arc_dijkstras::Graph<Eigen::Vector2d> graph(9);
    for (double x = -1; x <= 1; x += 1)
    {
        for (double y = -1; y <= 1; y += 1)
        {
            graph.AddNode(Eigen::Vector2d(x, y));
        }
    }

    // Y-direction edges
    graph.AddEdgeBetweenNodes(0, 1, 1.0);
    graph.AddEdgesBetweenNodes(1, 2, 1.0);
    graph.AddEdgesBetweenNodes(3, 4, 1.0);
    graph.AddEdgesBetweenNodes(4, 5, 1.0);
    graph.AddEdgesBetweenNodes(6, 7, 1.0);
    graph.AddEdgesBetweenNodes(7, 8, 1.0);

    // X-direction edges
    graph.AddEdgesBetweenNodes(0, 3, 1.0);
    graph.AddEdgesBetweenNodes(3, 6, 1.0);
    graph.AddEdgesBetweenNodes(1, 4, 1.0);
    graph.AddEdgesBetweenNodes(4, 7, 1.0);
    graph.AddEdgesBetweenNodes(2, 5, 1.0);
    graph.AddEdgesBetweenNodes(5, 8, 1.0);

    assert(graph.CheckGraphLinkage());


    auto dijkstras_result_4connected = arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector2d, std::allocator<Eigen::Vector2d>>::PerformDijkstrasAlgorithm(graph, 0);

    std::cout << "4-connected edges\n"
              << "Node index            : 0, 1, 2, 3, 4, 5, 6, 7, 8\n";
    std::cout << "Previous graph indices: " << PrettyPrint::PrettyPrint(dijkstras_result_4connected.second.first) << std::endl;
    std::cout << "Distance              : " << PrettyPrint::PrettyPrint(dijkstras_result_4connected.second.second) << std::endl;

    // Diagonal edges
    graph.AddEdgesBetweenNodes(0, 4, std::sqrt(2));
    graph.AddEdgesBetweenNodes(1, 5, std::sqrt(2));
    graph.AddEdgesBetweenNodes(3, 7, std::sqrt(2));
    graph.AddEdgesBetweenNodes(4, 8, std::sqrt(2));

    graph.AddEdgesBetweenNodes(1, 3, std::sqrt(2));
    graph.AddEdgesBetweenNodes(2, 4, std::sqrt(2));
    graph.AddEdgesBetweenNodes(4, 6, std::sqrt(2));
    graph.AddEdgesBetweenNodes(5, 7, std::sqrt(2));

    assert(graph.CheckGraphLinkage());
    auto dijkstras_result_8connected = arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector2d, std::allocator<Eigen::Vector2d>>::PerformDijkstrasAlgorithm(graph, 0);

    std::cout << "\n8-connected edges\n"
              << "Node index            : 0, 1, 2, 3, 4, 5, 6, 7, 8\n";
    std::cout << "Previous graph indices: " << PrettyPrint::PrettyPrint(dijkstras_result_8connected.second.first) << std::endl;
    std::cout << "Distance              : " << PrettyPrint::PrettyPrint(dijkstras_result_8connected.second.second) << std::endl;

    std::cout << "\nSerialization test... ";

    arc_dijkstras::Graph<Eigen::Vector2d> serialization_test_graph(2);
    serialization_test_graph.AddNode(Eigen::Vector2d(0,0));
    serialization_test_graph.AddNode(Eigen::Vector2d(1,1));
    serialization_test_graph.AddEdgesBetweenNodes(0, 1, 1.0);

    // Define the graph value serialization function
    const auto value_serializer_fn = [] (const Eigen::Vector2d& value, std::vector<uint8_t>& buffer)
    {
        const uint64_t start_buffer_size = buffer.size();
        uint64_t running_total = 0;

        running_total += arc_utilities::SerializeFixedSizePOD<double>(value(0), buffer);
        running_total += arc_utilities::SerializeFixedSizePOD<double>(value(1), buffer);

        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;

        assert(running_total == bytes_written);

        return bytes_written;
    };

    // Define the graph value serialization function
    const auto value_deserializer_fn = [] (const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        uint64_t current_position = current;

        // Deserialze 2 doubles
        std::pair<double, uint64_t> x = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
        current_position += x.second;
        std::pair<double, uint64_t> y = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
        current_position += y.second;

        const Eigen::Vector2d deserialized(x.first, y.first);

        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    };

    // Serialze the graph
    std::vector<uint8_t> buffer;
    serialization_test_graph.SerializeSelf(buffer, value_serializer_fn);

    auto deserialized_result = arc_dijkstras::Graph<Eigen::Vector2d>::Deserialize(buffer, 0, value_deserializer_fn);
    assert(deserialized_result.first.CheckGraphLinkage());

    std::cout << "passed" << std::endl;

    return 0;
}
