#ifndef TEMPLATE_UI_H
#define TEMPLATE_UI_H

#include <ncurses.h>
#include <unistd.h>

#include <map>
#include <string>
#include <string_view>
#include <vector>
class Point {
 public:
  Point(int i, int j) {
    i_ = i;
    j_ = j;
  }
  int GetI() const { return i_; };
  int GetJ() const { return j_; };
  void SetI(int i) { i_ = i; };
  void SetJ(int j) { j_ = j; };

 private:
  int i_;
  int j_;
};

class UI {
 public:
  UI();

  std::string GetHelloWorld();
  std::string GetLongText();

  void drawcircle(int i0, int j0, int radius);
  bool DrawLine(const Point &start, const Point &end,
                const std::string &str) const;

  bool DrawLine(const int i0, const int j0, const int i1, const int j1,
                const std::string &str) const;

  bool DrawSevenSegment(const int i0, const int j0, const int width,
                        const int height, const unsigned char char_code,
                        const std::string &str) const;

  void putpixel(const Point &pixel, const std::string &str,
                bool do_refresh = true) const {
    putpixel(pixel.GetI(), pixel.GetJ(), str.c_str(), do_refresh);
  }

  void putpixel(int i, int j, const std::string &str,
                bool do_refresh = true) const {
    mvwprintw(stdscr, i, j, "%s", str.c_str());
    if (do_refresh) {
      refresh();
    }
  }

  void PrintString(int i, int j, const std::string &str,
                   bool do_refresh = true) const {
    mvwprintw(stdscr, i, j, "%s", str.c_str());
    if (do_refresh) {
      refresh();
    }
  }

  void ScrollText(std::string_view text, int intervalMilliSeconds = 10,
                  int y = 0) {
    int window_x, window_y;
    getmaxyx(stdscr, window_y, window_x);

    int mid = window_x / 2;
    for (int i = 0; i <= text.size() / 2; ++i) {
      std::string left{text.substr(0, i)};
      std::string right{text.substr(text.size() - i)};
      std::string textToDisplay;
      if (i == text.size() / 2 && text.size() % 2 == 1) {
        textToDisplay = left + text[text.size() / 2] + right;
      } else {
        textToDisplay = left + right;
      }
      PrintString(y, mid - i, textToDisplay.c_str());

      refresh();                            // Print it on to the real screen
      usleep(1000 * intervalMilliSeconds);  // Sleep for 100 ms
    }
  }

  int ScrollLongText(std::string s, int intervalMilliSeconds = 10, int y = 0) {
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    std::string delimiter = "\n";
    size_t pos = 0;
    std::string token;
    int count = y;
    attron(COLOR_PAIR(3));  // Change color
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        ScrollText(token, /*intervalMilliSeconds=*/1, /*y=*/count);
        count+=1;
        s.erase(0, pos + delimiter.length());
    }
    ScrollText(s, /*intervalMilliSeconds=*/1, /*y=*/count);
    return count+1;
  }

  void PrintStringOnSevenSegment(int i, int j, int width, int height,
                                 int margin, const std::string &text,
                                 const std::string &str);

  std::map<unsigned char, unsigned char> char_map_;
};

#endif