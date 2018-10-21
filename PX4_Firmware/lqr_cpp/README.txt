README

Instruções para Fomulação do Modelo
===================================

Para configurar os parâmetros do seu problema será necessário editar os arquivos mpcdados.cpp, quadprog.cpp e simdados.cpp.

- lqrdados.cpp

	Configurar os valores das matrizes A, B, Cr, Horizonte de Predição N e os pesos Qy e Qu. A configuração deve ser feita diretamente no construtor da classe LQRDados::LQRDados() 

Instruções de Uso
=================

Dependências:
O programa foi escrito em C++ e testado no compilador GCC. Há uma dependência da biblioteca Eigen (http://eigen.tuxfamily.org) para a manipulação de matrizes.

Para Compilar:
Após efetuar o setup do ambiente, basta estar na pasta raiz do projeto e digitar o comando make.
> make

Para Executar:
Ainda na pasta raiz do projeto, digite:
> make run

Para Limpar o código compilado:
> make clean
