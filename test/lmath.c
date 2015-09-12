#include <stdio.h>
#include "../src/lmath.h"

main() {
  int i = 0, j = 0;

  double a[3][3] = {
    {1, 3, 5},
    {2, 4, 6},
    {9, 7, 8}
  };

  double b[3][3] = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
  };

  double c[3][3] = {
    {48, 57, 66},
    {60, 72, 84},
    {93, 117, 141}
  };

  double m[3][3] = {};

  double p = 98;

  double va[3] = {3, 5 ,7};
  double vb[3] = {4, 6, 8};
  double vc[3] = {-2, 4, -2};
  double vd[3] = {7, 11, 15};
  double ve[3] = {294, 490, 686};
  double vm[3] = {};

  matrix_multiply3(a, b, m);

  for(;i<3;++i) {
    for(j=0;j<3;++j) {
      if(m[i][j] != c[i][j])
        printf("%d,%d: %f != %f\n", i, j, m[i][j], c[i][j]);
      else
        printf("%d,%d: %f == %f\n", i, j, m[i][j], c[i][j]);
    }
  }

  if(p != vector_dot_product3(va, vb)) {
    printf("Vector dot product failed\n");
  } else {
    printf("OK\n");
  }


  vector_cross_product3(va, vb, vm);

  for(i=0;i<3;++i)
    if(vm[i] != vc[i])
      printf("%d: %f != %f\n", i, vm[i], vc[i]);
    else
      printf("%d: %f == %f\n", i, vm[i], vc[i]);

  vector_add3(va, vb, vm);

  for(i=0;i<3;++i)
    if(vm[i] != vd[i])
      printf("%d: %f != %f\n", i, vm[i], vd[i]);
    else
      printf("%d: %f == %f\n", i, vm[i], vd[i]);

  vector_scale3(va, p, vm);

  for(i=0;i<3;++i)
    if(vm[i] != ve[i])
      printf("%d: %f != %f\n", i, vm[i], ve[i]);
    else
      printf("%d: %f == %f\n", i, vm[i], ve[i]);
}
