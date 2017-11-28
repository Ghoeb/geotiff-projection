#pragma once

/** Representa un eje cartesiano segun el cual se divide el plano */
enum axis
{
	X,
	Y,
	Z
};

/** Representa un eje cartesiano segun el cual se divide el plano */
typedef enum axis Axis;

struct vector
{
  /** Componente X del vector */
  double X;
  /** Componente Y del vector */
  double Y;
  /** Componente Z del vector*/
  double Z;
};

/** Define un vector en R³ */
typedef struct vector Vector;

/** Obtiene una componente específica de un vector */
double vector_get_component(Vector v, Axis ax);

/** Crea un vector con las componentes dadas */
Vector vector_init(double X, double Y, double Z);

/** Obtiene la magnitud de un vector*/
double vector_size(Vector v);

/** Obtiene la magnitud al cuadrado de un vector */
double vector_size_squared(Vector v);

/** Normaliza un vector */
void   vector_normalize(Vector *v);

/** Entrega una versión normalizada de v, sin modificar a v.*/
Vector vector_normalized(Vector v);

/** Escala el vector para que ninguna componente pase del 1 */
void   vector_balance(Vector* v);

/** Restringe las componentes del vector al intervalo [inf, sup] */
void   vector_clamp(Vector *v, double inf, double sup);

/** Entrega el resultado de hacer clamp en el intervalo [inf, sup] */
Vector vector_clamped(Vector v, double inf, double sup);

/** Multiplica al vector por un escalar. */
void   vector_multiply_f(Vector *v, double c);

/** Entrega el resultado de multiplicar a v por c. */
Vector vector_multiplied_f(Vector v, double c);

/** Divide al vector por un escalar. */
void   vector_divide_f(Vector *v, double c);

/** Entrega el resultado de dividir a v en c. */
Vector vector_divided_f(Vector v, double c);

/** Multiplica los vectores componente a componente */
void   vector_multiply_v(Vector *v, Vector a);

/** Entrega el resultado de multiplicar los vectores componente a componente */
Vector vector_multiplied_v(Vector v1, Vector v2);

/** Le suma al vector otro vector */
void   vector_add_v(Vector *v, Vector a);

/** Entrega el resultado de sumar ambos vectores */
Vector vector_added_v(Vector v, Vector a);

/** Le resta a al vector v */
void   vector_substract_v(Vector *v, Vector a);

/** Entrega el resultado de restar ambos vectores */
Vector vector_substracted_v(Vector v, Vector a);

/** Entrega el resultado de restar ambos vectores (punteros) */
Vector vector_p_substracted_v(Vector const* v, Vector const* a);

/** Entrega el resultado del producto punto entre ambos vectores */
double vector_dot(Vector v1, Vector v2);

/** Entrega el resultado del producto cruz entre ambos vectores.*/
Vector vector_cross(Vector v1, Vector v2);

/** Combina 2 vectores de acuerdo al coeficiente entregado */
Vector vector_blend2(Vector v1, Vector v2, double u);

/** Combina 3 vectores de acuerdo a los coeficientes entregados */
Vector vector_blend3(Vector v1, Vector v2, Vector v3, double u, double v);

/** Obtiene el ángulo entre 2 vectores */
double vector_angle (Vector v1, Vector v2);

/** Impreime un vector por consola */
void   vector_print(Vector v);
