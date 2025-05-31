import time

from algorithm.generic_sr import GenericSR
from algorithm.segment_routing.equal_split_shortest_path import EqualSplitShortestPath
import networkx as nx

class TL_InverseCapacity(GenericSR):
    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        self.__nodes = nodes  # [i, ...]
        self.__links = links  # [(i,j,c), ...]
        self.__demands = demands  # {idx: (s,t,d), ...}
        self.__weights = None
        self.__waypoints = waypoints

    def solve(self) -> dict:
        """Verbesserte TLInverseCapacity mit volumenbasierter, normalisierter Belastung
        und iterativer Gewichts-Anpassung."""

        # add random waypoint for each demand
        t = time.process_time()
        pt_start = time.process_time()  # count process time (e.g. sleep excluded)

        # set link weights to inverse capacity scaled by max capacity
        max_c = max([c for _, _, c in self.__links])
        base_weights = {(i, j): max_c / c for i, j, c in self.__links}

        # NEU: Arbeitsgewichtskopie erstellen
        weights = base_weights.copy()

        # NEU: Straf-Faktor und Iterationen
        alpha = 0.3      # Straf-Faktor pro Einheit der normalisierten Last
        num_iter = 3     # Anzahl der Iterationsschritte

        # NEU: Iterative Last-Anpassung basierend auf normalisierter Last
        for _ in range(num_iter):
            # NEU: Volumenbasierte Lastinitialisierung
            link_load = {(i, j): 0.0 for i, j, _ in self.__links}

            # build graph with current weights
            G = nx.DiGraph()
            for (i, j, _) in self.__links:
                G.add_edge(i, j, weight=weights[(i, j)])

            # NEU: Jede Demand routen und Load akkumulieren
            for (s, t_dest, d) in self.__demands:
                try:
                    path = nx.shortest_path(G, source=s, target=t_dest, weight='weight')
                except nx.NetworkXNoPath:
                    continue
                for u, v in zip(path[:-1], path[1:]):
                    link_load[(u, v)] += d  # Volumen-basiert hinzufügen

            # NEU: Gewichte basierend auf normalisierter Last anpassen
            new_weights = {}
            for (i, j, c) in self.__links:
                base = base_weights[(i, j)]
                normalized = link_load[(i, j)] / c  # NEU: Last durch Kapazität teilen
                new_weights[(i, j)] = base * (1 + alpha * normalized)  # NEU: lineare Strafskala
            weights = new_weights

        # set final weights and perform equal-split shortest path routing
        self.__weights = weights
        post_processing = EqualSplitShortestPath(
            nodes=self.__nodes, links=self.__links, demands=self.__demands,
            split=True, weights=self.__weights, waypoints=self.__waypoints)
        solution = post_processing.solve()

        pt_duration = time.process_time() - pt_start
        exe_time = time.process_time() - t

        # update execution time
        solution["execution_time"] = exe_time
        solution["process_time"] = pt_duration
        return solution

    def get_name(self):
        """ returns name of algorithm """
        return f"tl_inverse_capacity"