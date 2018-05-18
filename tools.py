import wikipedia


def get_knowledge(term):
    """
    Get knowledge from Wikipedia (just two first sentences)

    :param term: Term to look up
    :type term: string
    :return: summary
    :rtype: string
    """
    summary = wikipedia.summary(term, sentences=2)
    return summary
