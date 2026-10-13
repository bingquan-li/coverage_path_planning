// Standalone: c++ -std=c++20 -O2 tests/reference_line_geometry_test.cc -o /tmp/reference_line_geometry_test
#define REFERENCE_LINE_GEOMETRY_ONLY
#include "../examples/rerun/reference_line.cc"
#include <cassert>
#include <iostream>

double cross(Point2D a, Point2D b, Point2D c)
{
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
bool intersects(Point2D a, Point2D b, Point2D c, Point2D d)
{
  constexpr double eps = 1e-10;
  auto on = [&](Point2D p, Point2D q, Point2D r) {
    return std::abs(cross(p,q,r)) <= eps &&
      r.x >= std::min(p.x,q.x)-eps && r.x <= std::max(p.x,q.x)+eps &&
      r.y >= std::min(p.y,q.y)-eps && r.y <= std::max(p.y,q.y)+eps;
  };
  return (cross(a,b,c)*cross(a,b,d)<0 && cross(c,d,a)*cross(c,d,b)<0) ||
    on(a,b,c) || on(a,b,d) || on(c,d,a) || on(c,d,b);
}
void check(std::vector<Point2D> const& p)
{
  auto g=compute_geometry(p);
  auto [l,r]=compute_widths(g.curvature,g.s,.05,2.,.8,g.sharp_corner);
  for(std::size_t i=0;i<p.size();++i) {
    assert(l[i]>0 && r[i]>0 && l[i]<=2 && r[i]<=2);
    if(!g.sharp_corner[i]) {
      assert(1-g.curvature[i]*l[i]>=.2-1e-10);
      assert(1+g.curvature[i]*r[i]>=.2-1e-10);
    }
    if(g.sharp_corner[i]) {
      std::cout<<"corner "<<i<<": left="<<l[i]<<", right="<<r[i]<<'\n';
      assert(l[i]>.5 && r[i]>.5);
    }
    if(i) {
      assert(std::abs(l[i]-l[i-1])<=.05*(g.s[i]-g.s[i-1])+1e-10);
      assert(std::abs(r[i]-r[i-1])<=.05*(g.s[i]-g.s[i-1])+1e-10);
    }
  }
  auto a=buildOffsetBoundary(p,l,1);
  auto b=buildOffsetBoundary(p,r,-1);
  for(auto const* curve : {&a,&b}) {
    for(std::size_t i=1;i<curve->size();++i) {
      assert(std::isfinite((*curve)[i].x) && std::isfinite((*curve)[i].y));
      for(std::size_t j=i+2;j<curve->size();++j)
        assert(!intersects((*curve)[i-1],(*curve)[i],(*curve)[j-1],(*curve)[j]));
    }
  }
  for(std::size_t i=1;i<a.size();++i)
    for(std::size_t j=1;j<b.size();++j)
      assert(!intersects(a[i-1],a[i],b[j-1],b[j]));
}
int main()
{
  check(generateSinePath(10,1,8,8,.8,.02));
  // Dense and sparse 90-degree corners, in both turning directions.
  for(double sign : {-1.,1.}) {
    check({{0,0},{10,0},{10,sign*10}});
    std::vector<Point2D> p;
    for(int i=0;i<=500;++i) p.push_back({i*.02,0});
    for(int i=1;i<=500;++i) p.push_back({10,sign*i*.02});
    check(p);
  }
  // A 160-degree turn exercises the bounded-miter bevel fallback.
  for(double sign : {-1.,1.})
    check({{0,0},{10,0},{10+10*std::cos(160*pi/180),sign*10*std::sin(160*pi/180)}});
  check(generateSinePath(10,1,8,8,-.8,.02));
  check({{0,0},{5,0},{10,0}});
  std::cout<<"All boundary checks passed\n";
}
