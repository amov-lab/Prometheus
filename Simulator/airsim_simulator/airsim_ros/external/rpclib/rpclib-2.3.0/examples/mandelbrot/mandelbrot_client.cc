#include <iostream>

#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"
#include "rpc/client.h"
#include "mandelbrot.h"

int main() {
    const int width = 1024, height = 768;

    rpc::client c("127.0.0.1", rpc::constants::DEFAULT_PORT);

    std::cout << "Calling get_mandelbrot asynchronically" << std::endl;
    auto result_obj = c.async_call("get_mandelbrot", width, height);

    std::cout << "Calling get_time synchronically" << std::endl;
    auto current_time = c.call("get_time").as<std::string>();
    std::cout << "Current time: " << current_time << std::endl;

    sf::Image image;
    image.create(width, height, sf::Color::Black);

    std::cout << "Waiting for get_mandelbrot result" << std::endl;
    auto result = result_obj.get().as<pixel_data>();
    std::cout << "Got mandelbrot data, displaying..." << std::endl;

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            auto item = result[x * height + y];
            auto color = sf::Color(item.r, item.g, item.b, 255);
            image.setPixel(x, y, color);
        }
    }

    sf::RenderWindow window(sf::VideoMode(width, height), "rpc mandelbrot client");

    sf::Texture texture;
    texture.loadFromImage(image, sf::IntRect(0, 0, width, height));
    sf::Sprite sprite;
    sprite.setTexture(texture, true);
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
        window.draw(sprite);
        window.display();
    }

    return 0;
}
