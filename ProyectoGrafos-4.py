#Librerías
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

opc = int(input("***GRAFOS***\nElige una opción:\n1.-Etiquetado y coloración de grafos.\n2.-Algortimo de Prim.\n3.-Algoritmo de Dijkstra.\n4.-Algoritmos de búsqueda.\nOpción: "))
if((opc>=1) and (opc<= 4)):
    if opc==1:
        def crear_grafo(matriz_nodos, lista_aristas):
            grafo = nx.Graph()

            # Agregar nodos
            for nodo in matriz_nodos:
                grafo.add_node(nodo)

            # Agregar aristas
            for arista in lista_aristas:
                grafo.add_edge(arista[0], arista[1])

            return grafo

        def obtener_orden_nodos(grafo):
            grados = dict(grafo.degree())
            orden_nodos = sorted(grados.keys(), key=lambda x: (-grados[x], x))
            return orden_nodos

        def colorear_grafo(grafo, orden_nodos):
            colores = {}
            for nodo in orden_nodos:
                colores_disponibles = set(range(len(grafo)))
                for vecino in grafo.neighbors(nodo):
                    if vecino in colores:
                        if colores[vecino] in colores_disponibles:
                            colores_disponibles.remove(colores[vecino])
                colores[nodo] = colores_disponibles.pop()
            lista_colores = [colores[nodo] for nodo in grafo.nodes()]
            return lista_colores

        # Solicitar al usuario la lista de nodos
        entrada_nodos = input("Ingresa la lista de nodos separados por espacios: ")

        # Convertir la entrada del usuario en una lista de nodos
        matriz_nodos = list(map(int, entrada_nodos.split()))

        # Solicitar al usuario la lista de aristas
        entrada_aristas = input("Ingresa la lista de aristas en el formato 'nodo1,nodo2 nodo3,nodo4': ")

        # Convertir la entrada del usuario en una lista de aristas
        lista_aristas = [tuple(map(int, arista.split(','))) for arista in entrada_aristas.split()]

        # Crear el grafo
        grafo = crear_grafo(matriz_nodos, lista_aristas)

        # Dibujar el grafo sin coloración
        nx.draw(grafo, with_labels=True)
        plt.title("Grafo sin coloración")
        plt.show()

        # Obtener el orden de los nodos
        orden_nodos = obtener_orden_nodos(grafo)

        # Imprimir el orden de los nodos
        print("Orden de nodos:")
        print(orden_nodos)

        # Colorear el grafo utilizando
        colores = colorear_grafo(grafo, orden_nodos)

        # Dibujar el grafo con coloración
        nx.draw(grafo, with_labels=True, node_color=colores, cmap=plt.cm.rainbow, vmin=0, vmax=max(colores))
        plt.title("Grafo con coloración")
        plt.show()
    elif opc==2:
        def prim(G, start_node):
            masinf = float('inf')
            A = {v: (None, masinf) for v in G.nodes}
            A[start_node] = (start_node, 0)

            aristas_buenas = set()
            nodos_conectados = {start_node}

            while A:
                mejor_peso = masinf
                mejor = None
                padre = None

                for v in nodos_conectados:
                    for u in G.neighbors(v):
                        peso_uv = G[v][u]['weight']
                        if u not in nodos_conectados and peso_uv < mejor_peso:
                            mejor = u
                            padre = v
                            mejor_peso = peso_uv

                if mejor is not None:
                    aristas_buenas.add((padre, mejor, mejor_peso))
                    nodos_conectados.add(mejor)

                A.pop(padre, None)

                if mejor is not None:
                    for v in G.neighbors(mejor):
                        if v in A:
                            peso_v = G[mejor][v]['weight']
                            if peso_v < A[v][1]:
                                A[v] = (mejor, peso_v)

                if mejor is None:
                    break  # Agregamos esta verificación para evitar un bucle infinito

            return list(aristas_buenas)


        # Solicitar al usuario que ingrese nodos y aristas
        nodos = list(map(int, input("Ingrese los nodos separados por espacios: ").split()))

        # Solicitar al usuario que ingrese las aristas en líneas separadas
        print("Ingrese las aristas en el formato nodo1 nodo2 peso (una arista por línea): ")
        aristas_input = []
        while True:
            arista = input().strip()
            if not arista:
                break
            aristas_input.append(tuple(map(int, arista.split())))

        # Solicitar al usuario que ingrese el nodo de inicio
        start_node = int(input("Ingrese el nodo de inicio: "))

        # Crear un grafo
        G = nx.Graph()
        G.add_nodes_from(nodos)
        G.add_weighted_edges_from(aristas_input)

        # Obtener el árbol de peso mínimo
        aristas_minimas = prim(G, start_node)

        # Crear el diseño
        KKL = nx.spring_layout(G)

        # Obtener etiquetas de aristas para la gráfica original
        labels = nx.get_edge_attributes(G, 'weight')

        # Crear una nueva figura
        plt.figure()
        # Dibujar la gráfica original con colores personalizados
        plt.title('GRAFO')
        pos = nx.spring_layout(G)  
        nx.draw(G, pos=pos, with_labels=True, node_color='skyblue', edge_color='gray', node_size=1000)
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels, font_size=10)
        plt.show()

        # Resaltar la ruta en la gráfica original
        ruta_aristas = [(u, v) for u, v, _ in aristas_minimas if u is not None and v is not None]
        nx.draw_networkx_edges(G, pos, edgelist=ruta_aristas, edge_color='red', width=2)

        # Crear un nuevo grafo con las aristas mínimas
        H = nx.Graph()
        H.add_weighted_edges_from(aristas_minimas)

        # Obtener etiquetas de aristas para el árbol de peso mínimo
        labels2 = nx.get_edge_attributes(H, 'weight')

        # Crear una nueva figura para la ruta
        plt.figure()
        # Dibujar la gráfica original de nuevo
        nx.draw(G, pos=pos, with_labels=True, node_color='skyblue', edge_color='gray', node_size=1000)
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels, font_size=10)
        # Resaltar la ruta en la gráfica original
        ruta_aristas = [(u, v) for u, v, _ in aristas_minimas if u is not None and v is not None]
        nx.draw_networkx_edges(G, pos, edgelist=ruta_aristas, edge_color='red', width=2)

        # Calcular el peso total del árbol
        peso_total = sum([H[u][v]['weight'] for u, v in H.edges()])
        # Agregar el peso total del árbol a la gráfica
        plt.text(0.5, -0.1, 'Ruta: {}\nPeso total: {}'.format(ruta_aristas, peso_total), horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
        plt.show()
    elif opc==3:
        def dijkstra(Grafo, salida, destino):
            dist, prev = {}, {}
            result = []

            for vertice in Grafo:
                dist[vertice] = float("inf")
                prev[vertice] = None
            dist[salida] = 0

            Q = set(Grafo.nodes)

            while Q:
                u = min(Q, key=lambda v: dist[v])
                Q.remove(u)
                result.append(u)

                for vecino in Grafo.neighbors(u):
                    peso = Grafo[u][vecino]['weight']
                    if vecino in Q and dist[vecino] > peso:
                        dist[vecino] = peso
                        prev[vecino] = u

            ruta = []
            actual = destino
            while actual is not None:
                ruta.insert(0, actual)
                actual = prev[actual]

            return ruta, dist, prev


        def construir_grafo_desde_entrada(nodos_input, aristas_input):
            grafo = nx.Graph()

            # Agregar nodos al grafo
            grafo.add_nodes_from(nodos_input)

            # Agregar aristas y pesos al grafo
            for arista in aristas_input:
                nodo_origen, nodo_destino, peso = arista
                grafo.add_edge(nodo_origen, nodo_destino, weight=int(peso))

            return grafo

        def graficar_grafo_con_ruta(G, ruta, layout):
            edge_labels = {(ruta[i], ruta[i + 1]): G[ruta[i]][ruta[i + 1]]['weight'] for i in range(len(ruta) - 1)}
            
            # Resaltar la ruta más corta
            edge_colors = ['red' if edge in edge_labels else 'gray' for edge in G.edges()]
            edge_widths = [2 if edge in edge_labels else 1 for edge in G.edges()]

            plt.title('Grafo con Ruta más Corta')
            nx.draw(G, layout, with_labels=True, node_color='skyblue', edge_color=edge_colors, width=edge_widths, node_size=500)
            nx.draw_networkx_edge_labels(G, layout, edge_labels=edge_labels, font_color='red')

            plt.show()

        # Solicitar al usuario que ingrese los nodos
        print("Ingrese los nodos separados por espacios:")
        nodos_input = input().split()

        # Solicitar al usuario que ingrese las aristas y pesos
        print("Ingrese las aristas en el formato 'nodo_origen nodo_destino peso' (una arista por línea)")
        aristas_input = [tuple(input().split()) for _ in range(int(input("Ingrese la cantidad de aristas: ")))]

        # Construir el grafo desde las entradas
        grafo = construir_grafo_desde_entrada(nodos_input, aristas_input)

        # Solicitar al usuario que ingrese el nodo de origen y destino
        nodo_origen = input("Ingrese el nodo de origen: ")
        nodo_destino = input("Ingrese el nodo de destino: ")

        # Aplicar el algoritmo de Dijkstra
        ruta_mas_corta, distancia, previos = dijkstra(grafo, nodo_origen, nodo_destino)

        # Imprimir resultados
        print(f"{ruta_mas_corta=}")
        print(f"{distancia=}")
        print(f"{previos=}")

        # Obtener el diseño para graficar
        KKL = nx.spring_layout(grafo)  # Puedes cambiar a otros algoritmos como circular_layout

        # Dibujar la gráfica original con colores personalizados
        plt.title('Grafo Original')
        nx.draw(grafo, KKL, with_labels=True, node_color='skyblue', edge_color='gray', node_size=500)
        plt.show()

        # Graficar la ruta más corta
        graficar_grafo_con_ruta(grafo, ruta_mas_corta, KKL)
    else:
        opc2 = int(input("***Algoritmos de Búsqueda***\nElige una opción:\n1.-Anchura.\n2.-Profundidad.\nOpción: "))
        if((opc2>=1) and (opc2<= 2)):
            if opc2==1:
                 # Solicitar al usuario que ingrese las aristas en el formato inicio fin (una arista por línea, dejar vacío para finalizar): 
                aristas_input = []
                print("Ingrese las aristas en el formato inicio fin (una arista por línea, dejar vacío para finalizar): ")
                while True:
                    arista = input().strip()
                    if not arista:
                        break
                    aristas_input.append(tuple(arista.split()))

                # Crear un grafo con NetworkX
                G = nx.Graph()
                G.add_edges_from(aristas_input)

                # Mostrar el grafo original
                print("Grafo original:")
                pos = nx.spring_layout(G, seed=42)  # Fijar la raiz para una disposición
                nx.draw(G, pos, with_labels=True, font_weight='bold', node_color='skyblue', edge_color='gray', node_size=500)
                plt.show()

                # Solicitar al usuario que ingrese el nodo de inicio y el nodo final
                inicio, final = input("Ingresa el nodo de inicio y el nodo final separados por espacio: ").split()

                # Realizar el recorrido en amplitud (BFS) y crear el árbol resultante
                arbol = nx.bfs_tree(G, inicio)
                # Realizar el recorrido en amplitud (BFS)
                bfs_nodes = list(nx.bfs_edges(G, inicio))

                # Obtener el camino más corto y colorear solo esas aristas
                camino_mas_corto = nx.shortest_path(G, inicio, final)
                aristas_camino_mas_corto = [(camino_mas_corto[n], camino_mas_corto[n + 1]) for n in range(len(camino_mas_corto) - 1)]

                # Crear una nueva gráfica con las aristas coloreadas para reflejar el recorrido en amplitud
                G_bfs = nx.Graph()
                G_bfs.add_edges_from(bfs_nodes)

                # Mostrar el grafo con las aristas coloreadas
                print("Grafo con aristas coloreadas por el recorrido en amplitud:")
                edge_colors = ['red' if edge in bfs_nodes or (edge[1], edge[0]) in bfs_nodes else 'gray' for edge in G.edges()]
                nx.draw(G, pos, with_labels=True, font_weight='bold', node_color='skyblue', edge_color=edge_colors, node_size=500)
                plt.show() 
                # Mostrar el árbol resultante
                print("\nÁrbol resultante del recorrido en amplitud:")
                pos_arbol = nx.planar_layout(arbol)
                nx.draw(arbol, pos_arbol, with_labels=True, font_weight='bold', node_color='lightcoral', edge_color='black', node_size=500)
                plt.show()
            else:
                def dfs_order(Grafo, nodo, destino, visitados=None, aristas_visitadas=None):
                    if visitados is None:
                        visitados = set()
                    if aristas_visitadas is None:
                        aristas_visitadas = set()

                    visitados.add(nodo)
                    print(f'Visitando nodo {nodo}')

                    if nodo == destino:
                        return  # Detenerse si se alcanza el nodo final

                    for vecino in sorted(Grafo.neighbors(nodo)):  # Orden alfabético o numérico
                        if vecino not in visitados:
                            aristas_visitadas.add((nodo, vecino))  # Añadir arista a la lista de aristas visitadas
                            dfs_order(Grafo, vecino, destino, visitados, aristas_visitadas)

                    return aristas_visitadas

                # Solicitar al usuario que ingrese las aristas en el formato inicio fin (una arista por línea, dejar vacío para finalizar):
                aristas_input = []
                print("Ingrese las aristas en el formato inicio fin (una arista por línea, dejar vacío para finalizar): ")
                while True:
                    arista = input().strip()
                    if not arista:
                        break
                    aristas_input.append(tuple(arista.split()))

                # Crear un grafo con NetworkX
                G = nx.Graph()
                G.add_edges_from(aristas_input)

                # Mostrar el grafo original
                print("Grafo original:")
                pos = nx.spring_layout(G, seed=42)  # Fijar la raíz para una disposición
                nx.draw(G, pos, with_labels=True, font_weight='bold', node_color='skyblue', edge_color='gray', node_size=500)
                plt.show()

                # Solicitar al usuario que ingrese el nodo de inicio y el nodo final
                inicio, final = input("Ingresa el nodo de inicio y el nodo final separados por espacio: ").split()

                # Realizar el recorrido en profundidad con orden alfabético o numérico
                visitados = set()
                aristas_visitadas = dfs_order(G, inicio, final, visitados)

                # Crear una nueva gráfica con el recorrido coloreado y detenerse en el nodo final
                G_dfs = nx.Graph()
                for u, v in G.edges():
                    if u in visitados and v in visitados:
                        G_dfs.add_edge(u, v)

                print("Grafo con aristas coloreadas por el recorrido en profundidad:")
                edge_colors = ['red' if (u, v) in aristas_visitadas or (v, u) in aristas_visitadas else 'gray' for u, v in G.edges()]
                nx.draw(G, pos, with_labels=True, font_weight='bold', cmap=plt.cm.rainbow, edge_color=edge_colors, node_size=500)
                plt.show()
else:
    print("Opción no valida.")