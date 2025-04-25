---
date: 27-01-2025 - 31-01-2025
course: linear_algebra
topic: data storage in matrixes
engagement: "5"
tags:
  - university
  - courses
  - linear_algebra
---
## 1. Class Sessions Summary

| Date       | Session Title | Key Concepts                                       | Resource Links |
| ---------- | ------------- | -------------------------------------------------- | -------------- |
| 2025-01-28 | Course Intro  | • Data in matrixes<br>• Data transformation        |                |
| 2025-01-30 | Lecture 1     | • Matrix basic concepts<br>• Matrix representation |                |

### 1.1 Estructura del Curso
**Módulo 1**: Matrices y sistemas de ecuaciones lineales.
- Método de Gauss y Gauss-Jordan.
- Sistemas homogéneos.
- Inversa de una matriz e inversa de una matriz de coeficientes.
- Regla de Crammer.
**Módulo 2**: Vectores en el plano y el espacio.
- *Proyecciones*: Producto punto y producto cruz.
- Aplicaciones con problemas afines a la ingeniería.
- *Espacio*: $\mathbb{R}^3$ (3 dimensiones).
**Módulo 3**: Espacios vectoriales.
- Definición.
- Subespacios.
- Bases y cambio de bases.
- Núcleo y rango de una matriz.
**Módulo 4**: Transformaciones lineales.
- Interpretaciones geométricas.
- Representación matricial de una trasnformación lineal.
**Módulo 5**: Valores y vectores propios.
- Cálculo de calores y vectores propios.
- Diagonalización.
- Aplicaciones con problemas afines a la ingeniería.
### 1.2 Actividades
1. Parcial (80%)
	- Semana 6 (21%)
	- Semana 11 (21%)
	- Semana 17 (30%)
2. Actividades (20%)
	- Quices.
	- Talleres UAO virtual. No tienen tiempo límite.
	- Tareas y actividades en grupo.
## 2. Conceptos Básicos de Matrices
Las matrices se denotan con letras mayúsculas. Tamaño: _filas x columnas_.
**Tipos de Matrices**:
- *Triangular superior e inferior*: Matriz cuadrada cuyos elementos encima (o debajo) de su diagonal son ceros.
- *Nula*: Todos sus elementos son ceros.
- *Matriz fila y columna*: Una sola fila y una sola columna respectivamente.
- *Cuadrada*: Igual número de filas y columnas.
- *Matriz diagonal*: Los elementos por fuera de su diagonal principal son ceros.
	- *Matriz identidad*. Los elementos de la diagonal principal son unos.
### 2.1 Operaciones con Matrices
*Suma/resta*: Únicamente con matrices de igual tamaño.
*Multiplicación*: Con matrices $A_{m\times n}$ y $B_{n\times l}$ para $(A\times B)_{m\times l}$
	
	$A\times B=\left[\begin{matrix}a_{1\times 1}&a_{2\times 1}\\a_{1\times 2}&a_{2\times 2}\\\end{matrix}\right]\times \left[\begin{matrix}b_{1\times 1}&b_{2\times 1}\\b_{1\times 2}&b_{2\times 2}\\\end{matrix}\right]=\left[\begin{matrix}a_{1\times1}\times b_{1\times1}+\ a_{1\times2}\times b_{2\times1}&a_{1\times2}\times b_{1\times2}+\ a_{1\times2}\times b_{2\times2}\\a_{2\times1}\times b_{1\times1}+\ a_{2\times2}\times b_{2\times1}&a_{2\times1}\times b_{1\times2}+\ a_{2\times2}\times b_{2\times2}\\\end{matrix}\right]$

*Transposición*: Intercambiar filas y columnas. $A_{m\times n}\rightarrow A^T_{n\times m}$

### 2.2 Determinante de una Matriz
- Si una matriz tiene dos filas o columnas iguales, su determinante es cero.
- Si una matriz tiene una fila o columna de ceros, su determinante es cero.
- Si una matriz es triangular, su determinante es el producto de los elementos de su diagonal.
- El determinante de $A$ y $A^T$ es el mismo.
### 2.2.1 Inversa de una Matriz
- $A$ es invertible sí y solo sí $det{A}\neq0$  
	- $A^{-1}\times A = I$  
	- $A\times x = b\rightarrow A^{-1}\times b = x$