package com.example.davidedidonato.jobride.algorithm.bestPath

import android.util.Log
import com.example.davidedidonato.jobride.data.user.User
import com.example.davidedidonato.jobride.util.AppExecutors


// In this Traveling Salesman Problem variant, the ending address can be different from the starting one
object TSPVariantAlgorithm {

    fun findBestPath(appExecutors: AppExecutors, startingNode: Node?, intermediateNodes: List<Node>, endingNode: Node?, onComplete: (nodes: List<Node>) -> Unit) {
        appExecutors.diskIO.execute {
            val bestPath: List<BestPathNode>
            val graph = createGraph(startingNode, intermediateNodes, endingNode)
            bestPath = searchBestPath(startingNode, graph, endingNode)
            onComplete(convertToList(bestPath))
        }
    }


    /**
     *
     * INIZIO BLOCCO DI CREAZIONE DEL GRAFO
     *
     */
    private fun createGraph(startingNode: Node?, intermediateNodes: List<Node>, endingNode: Node?): ArrayList<BestPathNode> {
        val graph = ArrayList<BestPathNode>()
        startingNode?.also { graph.add(BestPathNode(startingNode, HashMap())) }
        for (n in intermediateNodes) {
            graph.add(BestPathNode(n, HashMap()))
        }
        endingNode?.also { graph.add(BestPathNode(endingNode, HashMap())) }
        for (n1 in graph) {
            for (n2 in graph) {
                if (n1 != n2) {
                    n1.distances[n2] = distanceBetweenCoordinates(n1.node.getCoordinates().first, n1.node.getCoordinates().second,
                        n2.node.getCoordinates().first, n2.node.getCoordinates().second)
                }
            }
        }
        return graph
    }

    private fun distanceBetweenCoordinates(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val earthRadiusKm = 6371

        val dLat = Math.toRadians(lat2-lat1)
        val dLon = Math.toRadians(lon2-lon1)

        val newLat1 = Math.toRadians(lat1)
        val newLat2 = Math.toRadians(lat2)

        val a = Math.sin(dLat/2) * Math.sin(dLat/2) + Math.sin(dLon/2) * Math.sin(dLon/2) * Math.cos(newLat1) * Math.cos(newLat2)
        val c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a))
        return earthRadiusKm * c
    }
    /**
     *
     * FINE BLOCCO DI CREAZIONE DEL GRAFO
     *
     */


    /**
     *
     * INIZIO BLOCCO DI RICERCA DEL PERCORSO MIGLIORE
     *
     */
    private fun searchBestPath(startingNode: Node?, graph: ArrayList<BestPathNode>, endingNode: Node?): List<BestPathNode> {
        var startingBestPathNode : BestPathNode? = null
        var endingBestPathNode : BestPathNode? = null
        var startingIndex: Int? = null
        var endingIndex: Int? = null
        for (i in 0..graph.lastIndex) {
            if (graph[i].node == startingNode) {
                startingIndex = i
                startingBestPathNode = graph[i]

            }
            if (graph[i].node == endingNode) {
                endingIndex = i
                endingBestPathNode = graph[i]
            }
        }
        startingIndex?.run { graph.removeAt(startingIndex) }
        endingIndex?.run { graph.removeAt(endingIndex) }
        return depthFirstSearch(startingBestPathNode, graph, endingBestPathNode)
    }

    private fun depthFirstSearch(startingBestPathNode: BestPathNode?, availableBestPathNodes: MutableList<BestPathNode>, endingBestPathNode: BestPathNode?) : List<BestPathNode> {
        val currentlyAvailableNodes = availableBestPathNodes.toMutableList()
        val previouslyAvailableNodes = HashMap<Int, ArrayList<BestPathNode>>()
        val currentPath = ArrayList<BestPathNode>()
        var bestPath : ArrayList<BestPathNode> = ArrayList()
        var bestDistance = Double.MAX_VALUE

        while (!currentlyAvailableNodes.isEmpty() || !previouslyAvailableNodes.isEmpty()) {
            // Se ci sono ancora nodi assegnabili procedo normalmente
            if (!currentlyAvailableNodes.isEmpty()) {
                // Prendo il primo nodo assegnabile tra quelli disponibili rimuovendolo dai nodi assegnabili successivamente
                val chosenNode = currentlyAvailableNodes.removeAt(currentlyAvailableNodes.lastIndex)

                // Lo aggiungo al percorso corrente
                currentPath.add(chosenNode)

                if(currentlyAvailableNodes.size > 0) {
                    // Inizializzo la struttura associata al nodo assegnato per fare backtracking
                    previouslyAvailableNodes[currentPath.lastIndex] = ArrayList()
                    // Assegno tutti i nodi tranne quello assegnato al percorso corrente alla struttura per fare backtracking
                    currentlyAvailableNodes.forEach {
                        previouslyAvailableNodes[currentPath.lastIndex]?.add(it)
                    }
                }
            }
            else { // Altrimenti bisogna fare backtracking
                // Cerco quindi il primo indice nel percorso corrente (partendo dalle foglie) per cui è possibile il backtracking
                for (i in (currentPath.lastIndex - 1) downTo 0) {
                    var chosenBestPathNode: BestPathNode? = null
                    previouslyAvailableNodes[i]?.let {
                        // A quel punto, prendo l'ultimo tra i nodi non scelti per l'indice considerato
                        chosenBestPathNode = it.removeAt(it.lastIndex)

                        if(it.isNullOrEmpty()) previouslyAvailableNodes.remove(i)

                        // Aggiungendo alle possibilità per le scelte future i nodi che rimuovo dall'albero per il backtracking
                        while(!currentPath.isEmpty() && i != currentPath.lastIndex + 1) {
                            val removedNode = currentPath.removeAt(i)
                            if (removedNode != chosenBestPathNode) {
                                currentlyAvailableNodes.add(removedNode)
                            }
                        }
                        // E infine aggiungo il nodo al percorso corrente
                        currentPath.add(chosenBestPathNode!!)
                    }
                    if (chosenBestPathNode != null) break
                }
            }
            // Se il percorso è costituito da 4 nodi, allora è completo e si può calcolare la distanza
            if (currentPath.size == availableBestPathNodes.size) {
                val tempDistanceFirst = calculateTotalDistance(startingBestPathNode, currentPath, endingBestPathNode)
                val tempDistanceSecond = calculateTotalDistance(endingBestPathNode, currentPath, startingBestPathNode)

                if (tempDistanceFirst < bestDistance || tempDistanceSecond < bestDistance) {
                    bestPath = ArrayList(currentPath)
                    if (tempDistanceFirst < bestDistance && tempDistanceFirst <= tempDistanceSecond) {
                        startingBestPathNode?.also { bestPath.add(0, it) }
                        endingBestPathNode?.also { bestPath.add(it) }
                        bestDistance = tempDistanceFirst
                    }
                    else if (tempDistanceSecond < bestDistance && tempDistanceSecond < tempDistanceFirst) {
                        startingBestPathNode?.also { bestPath.add(it) }
                        endingBestPathNode?.also { bestPath.add(0, it) }
                        bestDistance = tempDistanceSecond
                    }
                }
            }
        }
        if (bestPath.indexOf(startingBestPathNode) == bestPath.lastIndex || bestPath.indexOf(endingBestPathNode) == 0) {
            bestPath.reverse()
        }
        Log.d("TSPV Final", printPath(bestPath) + "; la distanza e' " + bestDistance)
        return bestPath
    }

    private fun printPath(currentPath: List<BestPathNode>): String {
        var result = "Il percorso corrente e' "
        for (n in currentPath) {
            (n.node as User).run {
                result += "" + this.startingAddress + " -> "
            }
        }
        result += " FINE"
        return result
    }

    private fun calculateTotalDistance(starting: BestPathNode?, graph: List<BestPathNode>, ending: BestPathNode?): Double {
        var result = 0.00
        starting?.also { result += it.distances[graph.first()]!! }
        ending?.also { result += it.distances[graph.last()]!! }
        graph.forEachIndexed { index, node ->
            if (node != graph.last()) {
                result += node.distances[graph[index + 1]]!!
            }
        }
        return result
    }
    /**
     *
     * FINE BLOCCO DI RICERCA DEL PERCORSO MIGLIORE
     *
     */


    private fun convertToList(bestPath: List<BestPathNode>): List<Node> {
        val result: ArrayList<Node> = ArrayList()
        for (n in bestPath) {
            result.add(n.node)
        }
        return result
    }
}