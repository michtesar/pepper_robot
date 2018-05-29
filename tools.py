import wikipedia
import chatterbot

chatbot = None


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


def chatbot_init():
    chatbot = chatterbot.ChatBot(
        'Pepper Chatbot',
        trainer='chatterbot.trainers.ChatterBotCorpusTrainer'
    )
    #chatbot.train("chatterbot.corpus.english")


def chatbot_ask(question):
    answer = chatbot.get_response(question)
    return answer
