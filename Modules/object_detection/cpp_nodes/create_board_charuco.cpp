/***************************************************************************************************************************
 * create_board_charuco.cpp
 * Author: Jario
 * Update Time: 2020.12.05
 *
***************************************************************************************************************************/

#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>

using namespace cv;

namespace {
const char* about = "Create a ChArUco board image";
const char* keys  =
        "{@outfile |<none> | Output image }"
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in pixels) }"
        "{ml       |       | Marker side length (in pixels) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{m        |       | Margins size (in pixels). Default is (squareLength-markerLength) }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }";
}

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 7) {
        parser.printMessage();
        return 0;
    }

    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    int squareLength = parser.get<int>("sl");
    int markerLength = parser.get<int>("ml");
    int dictionaryId = parser.get<int>("d");
    int margins = squareLength - markerLength;
    if(parser.has("m")) {
        margins = parser.get<int>("m");
    }

    int borderBits = parser.get<int>("bb");
    bool showImage = parser.get<bool>("si");

    String out = parser.get<String>(0);

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Size imageSize;
    imageSize.width = squaresX * squareLength + 2 * margins;
    imageSize.height = squaresY * squareLength + 2 * margins;

    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength,
                                                            (float)markerLength, dictionary);

    // show created board
    Mat boardImage;
    board->draw(imageSize, boardImage, margins, borderBits);

    if(showImage) {
        imshow("board", boardImage);
        waitKey(0);
    }

    imwrite(out, boardImage);

    return 0;
}
