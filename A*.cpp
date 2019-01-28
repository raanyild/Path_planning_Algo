#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<queue>
#include<limits>
#include<cmath>

using namespace cv;
using namespace std;

struct pt
{
  int x, y;
  float dist = numeric_limits<float>::max();
  float preference = numeric_limits<float>::max();
  bool visited = false;
  bool hindrance  = false;
  struct pt *parent_node = NULL;
};

//operator overloading for the pt struct for <
bool operator<(const pt& p1, const pt& p2)
{
    return p1.preference > p2.preference;
}

bool isPoint(int x, int y, int r, int c)
{
  if((x >= 0 && x < r) && (y >= 0 && y < c))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool isReached(int x, int y, int r, int c)
{
  if ((x == r-1) && (y == c-1))
  {
    return(true);
  }
  else
  {
    return (false);
  }
}

//set preference function for desired destination
float heuristic(int x, int y, int r, int c)
{
  float d = abs(x - y);
  float alpha = 5;/*set the value*/
  return d*alpha;
}

//A* function
void pathFinder(vector<vector<pt> > &matrix, pt source, Mat &img)
{
  int r = img.rows;
  int c = img.cols;
  priority_queue<pt> l;
  l.push(source);
  matrix[source.x][source.y].dist = 0;
  matrix[source.x][source.y].preference = heuristic(source.x, source.y, r, c) ;
  img.at<Vec3b>(source.x, source.y) = {79, 79, 47};
  namedWindow("pathFinder", WINDOW_NORMAL);
  while(!l.empty())
  {
    pt p = l.top();
    matrix[p.x][p.y].visited = true;
    l.pop();
    //move down
    if(isPoint(p.x + 1, p.y, r, c))
    {
      if(isReached(p.x + 1, p.y, r, c))
      {
        matrix[p.x + 1][p.y].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x + 1][p.y].hindrance) && !(matrix[p.x + 1][p.y].visited))
      {
          float cost = matrix[p.x][p.y].dist + 1;
          if(cost < matrix[p.x + 1][p.y].dist)
          {
            img.at<Vec3b>(p.x + 1, p.y) = {79, 79, 47};
            matrix[p.x + 1][p.y].dist = cost;
            matrix[p.x + 1][p.y].preference = cost + heuristic(p.x + 1, p.y, r, c);
            l.push(matrix[p.x + 1][p.y]);
            matrix[p.x + 1][p.y].parent_node = &matrix[p.x][p.y];
          }
      }
    }
    //move right
    if(isPoint(p.x, p.y + 1, r, c))
    {
      if(isReached(p.x, p.y + 1, r, c))
      {
        matrix[p.x][p.y + 1].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x][p.y + 1].hindrance) && !(matrix[p.x][p.y + 1].visited))
      {
        float cost = matrix[p.x][p.y].dist + 1;
        if(cost < matrix[p.x][p.y + 1].dist)
        {
          img.at<Vec3b>(p.x, p.y + 1) = {79, 79, 47};
          matrix[p.x][p.y + 1].dist = cost;
          matrix[p.x][p.y + 1].preference = cost + heuristic(p.x, p.y + 1, r, c);
          l.push(matrix[p.x][p.y + 1]);
          matrix[p.x][p.y + 1].parent_node = &matrix[p.x][p.y];
        }
      }
    }
    //move top
    if(isPoint(p.x - 1, p.y, r, c))
    {
      if(isReached(p.x - 1, p.y, r, c))
      {
        matrix[p.x - 1][p.y].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x - 1][p.y].hindrance) && !(matrix[p.x - 1][p.y].visited))
      {
        float cost = matrix[p.x][p.y].dist + 1;
        if(cost < matrix[p.x - 1][p.y].dist)
        {
          img.at<Vec3b>(p.x - 1, p.y) = {79, 79, 47};
          matrix[p.x - 1][p.y].dist = cost;
          matrix[p.x - 1][p.y].preference = cost + heuristic(p.x - 1, p.y, r, c);
          l.push(matrix[p.x - 1][p.y]);
          matrix[p.x - 1][p.y].parent_node = &matrix[p.x][p.y];
        }
      }
    }
    //move left
    if(isPoint(p.x, p.y - 1, r, c))
    {
      if(isReached(p.x, p.y - 1, r, c))
      {
        matrix[p.x][p.y - 1].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x][p.y - 1].hindrance) && !(matrix[p.x][p.y - 1].visited))
      {
        float cost = matrix[p.x][p.y].dist + 1;
        if(cost < matrix[p.x][p.y - 1].dist)
        {
          img.at<Vec3b>(p.x, p.y - 1) = {79, 79, 47};
          matrix[p.x][p.y - 1].dist = cost;
          matrix[p.x][p.y - 1].preference = cost + heuristic(p.x, p.y - 1, r, c);
          l.push(matrix[p.x][p.y - 1]);
          matrix[p.x][p.y - 1].parent_node = &matrix[p.x][p.y];
        }
      }
    }
    //move down-right
    if(isPoint(p.x + 1, p.y + 1, r, c))
    {
      if(isReached(p.x + 1, p.y + 1, r, c))
      {
        matrix[p.x + 1][p.y + 1].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x + 1][p.y + 1].hindrance) && !(matrix[p.x + 1][p.y + 1].visited))
      {
        float cost = matrix[p.x][p.y].dist + 1.414;
        if(cost < matrix[p.x + 1][p.y + 1].dist)
        {
          img.at<Vec3b>(p.x + 1, p.y + 1) = {79, 79, 47};
          matrix[p.x + 1][p.y + 1].dist = cost;
          matrix[p.x + 1][p.y + 1].preference = cost + heuristic(p.x + 1, p.y + 1, r, c);
          l.push(matrix[p.x + 1][p.y + 1]);
          matrix[p.x + 1][p.y + 1].parent_node = &matrix[p.x][p.y];
        }
      }
    }
    //move right-top
    if(isPoint(p.x - 1, p.y + 1, r, c))
    {
      if(isReached(p.x - 1, p.y + 1, r, c))
      {
        matrix[p.x - 1][p.y + 1].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x - 1][p.y + 1].hindrance) && !(matrix[p.x - 1][p.y + 1].visited))
      {
        float cost = matrix[p.x][p.y].dist + 1.414;
        if(cost < matrix[p.x - 1][p.y + 1].dist)
        {
          img.at<Vec3b>(p.x - 1, p.y + 1) = {79, 79, 47};
          matrix[p.x - 1][p.y + 1].dist = cost;
          matrix[p.x - 1][p.y + 1].preference = cost + heuristic(p.x - 1, p.y + 1, r, c);
          l.push(matrix[p.x - 1][p.y + 1]);
          matrix[p.x - 1][p.y + 1].parent_node = &matrix[p.x][p.y];
        }
      }
    }
    //move left-down
    if(isPoint(p.x + 1, p.y - 1, r, c))
    {
      if(isReached(p.x + 1, p.y - 1, r, c))
      {
        matrix[p.x + 1][p.y - 1].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x + 1][p.y - 1].hindrance) && !(matrix[p.x + 1][p.y - 1].visited))
      {
        float cost = matrix[p.x][p.y].dist + 1.414;
        if(cost < matrix[p.x + 1][p.y - 1].dist)
        {
          img.at<Vec3b>(p.x + 1, p.y - 1) = {79, 79, 47};
          matrix[p.x + 1][p.y - 1].dist = cost;
          matrix[p.x + 1][p.y - 1].preference = cost + heuristic(p.x + 1, p.y - 1, r, c);
          l.push(matrix[p.x + 1][p.y - 1]);
          matrix[p.x + 1][p.y - 1].parent_node = &matrix[p.x][p.y];
        }
      }
    }
    //move top-left
    if(isPoint(p.x - 1, p.y - 1, r, c))
    {
      if(isReached(p.x - 1, p.y - 1, r, c))
      {
        matrix[p.x - 1][p.y - 1].parent_node = &matrix[p.x][p.y];
        break;
      }
      if(!(matrix[p.x - 1][p.y - 1].hindrance) && !(matrix[p.x - 1][p.y - 1].visited))
      {
        float cost = matrix[p.x][p.y].dist + 1.414;
        if(cost < matrix[p.x - 1][p.y - 1].dist)
        {
          img.at<Vec3b>(p.x - 1, p.y - 1) = {79, 79, 47};
          matrix[p.x - 1][p.y - 1].dist = cost;
          matrix[p.x - 1][p.y - 1].preference = cost + heuristic(p.x - 1, p.y - 1, r, c);
          l.push(matrix[p.x - 1][p.y - 1]);
          matrix[p.x - 1][p.y - 1].parent_node = &matrix[p.x][p.y];
        }
      }
    }
    imshow("pathFinder", img);
    char stop = waitKey(1);
    if(stop == ' ')
    {
      break;
    }
  }
  img.at<Vec3b>(r-1, c-1) = {0, 0, 255};
  pt *index_node;
  index_node = matrix[r-1][c-1].parent_node;
  while(1)
  {
    pt index = *index_node;
    img.at<Vec3b>(index.x, index.y) = {0, 0, 255};
    if(index.x == 0 && index.y == 0)
    {
      break;
    }
    index_node = index.parent_node;
  }
  imshow("pathFinder", img);
  char stop = waitKey(0);
  if(stop == ' ')
  {
    return;
  }
  return;
}

int main()
{
  Mat img = imread("root.jpg",1);
  vector<vector<pt> > matrix(img.rows);
  for (int k = 0; k < img.rows; k++)
  {
    matrix[k] = vector<pt>(img.cols);
  }
  Mat gray;
  cvtColor(img, gray, COLOR_RGB2GRAY);
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      matrix[i][j].x = i;
      matrix[i][j].y = j;
      if(gray.at<uchar>(i,j) < 50)
      {
        matrix[i][j].hindrance = true;
      }
    }
  }
  if(!matrix[0][0].hindrance)
  {
    pathFinder(matrix, matrix[0][0], img);
  }
}
