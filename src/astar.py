#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import heapq as pq


class ASTAR(GridMap):
    def __init__(self):
        super(ASTAR, self).__init__()
        self.position = position
        self.parent = parent
        self.g = 0
        self.f = 0
        
    def heapsort(self, list_in):                                                                      # metoda sortujaca liste do odwiedzenia
        h = []
        for val in list_in:
            pq.heappush(h, val)
        return [pq.heappop(h) for i in range(len(h))]

    def heuristics(self, pos):
        distance = ((pos[1] - end[1]) ** 2) + ((pos[0] - end[0]) ** 2)
        return distance

    def search(self):
        start = self.start
        end = self.end
        start_n = ASTAR(None, start)
        start_n.g = 0
        start_n.f = dfs.heuristics(start_n.position, end)
        end_n = ASTAR(None, end)
        end_n.g = end_n.f = 0
        visited = []
        toVisit = [(start_n.f, start_n)]                                                                # dodaje punkt startowy do listy do odwiedzenia
        path = []
        neighbours = [(0, -1), (0, 1), (-1, 0), (1, 0)]                                                 # okreslam czterosasiedztwo

        while len(toVisit) > 0:
            actual_n = toVisit[0][1]                                                                    # ustawiam aktualny wierzcholek

            toVisit = dfs.heapsort(toVisit)                                                             # sortuje liste do odwiedzenia, tak by wierzcholek z najmniejsza wartoscia F byl na poczatku
            if toVisit[0][0] < actual_n.f:                                                              # jezeli wierzcholek na poczatku listy ma mniejsza wartosc F niz obecny, zostaje ustawiony jako aktualny
                actual_n = toVisit[0][1]

            toVisit.pop(0)                                                                              # usuwam aktualny wierzcholek z listy do odwiedzenia
            visited.append(actual_n)                                                                    # dodaje aktualny wierzcholek do odwiedzonych

            self.map.data[actual_n.position[1] + actual_n.position[0] * self.map.info.width] = 50       # zmieniam wartosc komorki mapy na "odwiedzona" i publikuje mape
            self.publish_visited()
            if actual_n.position == end_n.position:                                                     # jezeli aktualny wierzcholek jest koncowym, odtwarzam sciezke
                while actual_n is not None:
                    path.append(actual_n.position)                                                      # dodaje rodzicow wierzcholka do sciezki
                    actual_n = actual_n.parent                                                          # ustawiam rodzica aktualnego wierzcholka jako aktualny w celu sprawdzenia jego rodzica
                path = path[::-1]                                                                       # odwracam sciezke, aby prowadzila od poczatku do konca
                print("Znaleziono rozwiazanie!")
                break

            children = []
            for neigh in neighbours:
                neigh_pos = (actual_n.position[0] + neigh[1], actual_n.position[1] + neigh[0])          # uzyskanie pozycji sasiada
                if self.map.data[neigh_pos[1] + neigh_pos[0] * self.map.info.width] != 100:             # jezeli sasiad nie jest sciana, stworz nowy obiekt i dodaj go do listy wierzcholkow potomnych
                    new_n = ASTAR(actual_n, neigh_pos)
                    children.append(new_n)
            for child in children:
                if self.map.data[child.position[1] + child.position[0] * self.map.info.width] == 50:    # jezeli wierzcholek potomny byl odwiedzony, sprawdz nastepnego
                    continue
                child.g = actual_n.g + 1                                                                # jezeli wierzcholek potomny nie byl odwiedzony, oblicz wartosi G i F
                child.f = child.g + dfs.heuristics(child.position, end)
                for vis in toVisit:
                    if child == vis and child.g > vis.g:                                                # jezeli wierzcholek potomny znajduje sie na liscie do odwiedzenia, a jego obecna sziezka jest dluzsza niz odkryta wczesniej, zbadaj nastepny wierzcholek
                        continue
                toVisit.append((child.f, child))                                                        # jezeli wierzcholka nie ma na liscie do odwiedzenia lub jest to krotsza sciezka, dopisz do odwiedzenia

        self.publish_visited()
        self.publish_path(path)


if __name__ == '__main__':
    dfs = ASTAR()
    dfs.search()
