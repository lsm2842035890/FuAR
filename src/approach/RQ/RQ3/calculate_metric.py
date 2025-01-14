import os
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'expandable_segments:True'
import json
import nltk
import numpy as np
from nltk.translate.bleu_score import corpus_bleu
from nltk.translate.meteor_score import meteor_score
from rouge_score import rouge_scorer
from bert_score import score as bert_score
from bleurt import score as bleurt_score
from transformers import AutoModel, AutoTokenizer,AutoModelForSequenceClassification
import torch
# print(nltk.data.path)
# Ensure nltk data is downloaded
# nltk.download('punkt')
nltk.data.path.append('/home/lsm/nltk_data')
# print("NLTK data downloaded")
# print(nltk.word_tokenize("Hello \n, world!"))
# Load BLEURT model (requires pre-trained BLEURT model)
from bleurt import score as bleurt_score
# # 指定模型名称
# model_name = "bleurt-base-128"

# # 指定下载路径
# cache_dir = "/home/lsm/SFTSG_NME/src/approach/RQ/RQ3"

# # 下载和加载BLEURT模型和分词器，并指定下载路径
# model = AutoModel.from_pretrained(model_name, cache_dir=cache_dir)
# tokenizer = AutoTokenizer.from_pretrained(model_name, cache_dir=cache_dir)

# # 输出加载的模型路径
# print(f"Model saved to: {cache_dir}")
# bleurt_scorer = bleurt_score.BleurtScorer(model_path = '/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/bleurt-base-128')

# Define a function to read the content of a file
def read_file(file_path):
    """
    Read a file, supporting both .txt and .json formats.
    """
    file_extension = file_path.split('.')[-1].lower()

    if file_extension == 'txt':
        # For .txt files, read as plain text
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read().strip()

    elif file_extension == 'json':
        # For .json files, load the JSON and return it as a string
        with open(file_path, 'r', encoding='utf-8') as f:
            json_data = json.load(f)
            return json.dumps(json_data)  # Convert JSON data to string

    else:
        raise ValueError(f"Unsupported file format: {file_extension}")

# Define a function to calculate BLEU-4
def calculate_bleu(reference_texts, generated_texts):
    references = [nltk.word_tokenize(ref) for ref in reference_texts]
    # print(references)
    hypotheses = [nltk.word_tokenize(gen) for gen in generated_texts]
    references = [[ref] for ref in references]
    # print(hypotheses)
    bleu_score = corpus_bleu(references, hypotheses, weights=(0.25, 0.25, 0.25, 0.25))
    # print(bleu_score)
    return bleu_score

# Define a function to calculate METEOR
def calculate_meteor(reference_texts, generated_texts):
    references = [nltk.word_tokenize(ref) for ref in reference_texts]
    hypotheses = [nltk.word_tokenize(gen) for gen in generated_texts]
    scores = [meteor_score([ref], gen) for ref, gen in zip(references, hypotheses)]
    # print(np.mean(scores))
    return np.mean(scores)

# Define a function to calculate ROUGE-L
def calculate_rouge(reference_texts, generated_texts):
    references = [nltk.word_tokenize(ref) for ref in reference_texts]
    hypotheses = [nltk.word_tokenize(gen) for gen in generated_texts]
    rouge = rouge_scorer.RougeScorer(['rougeL'], use_stemmer=True)
    # scores = [rouge.score(ref, gen)['rougeL'].fmeasure for ref, gen in zip(references, hypotheses)]
    scores = [rouge.score(' '.join(ref), ' '.join(gen))['rougeL'].fmeasure for ref, gen in zip(references, hypotheses)]
    # print(np.mean(scores))
    return np.mean(scores)

# Define a function to calculate BERTScore
def calculate_bertscore(reference_texts, generated_texts):
    P, R, F1 = bert_score(reference_texts, generated_texts, model_type= 'roberta-base',lang='en', rescale_with_baseline=True)
    # print (np.mean(F1.numpy()))
    return np.mean(F1.numpy())

# Define a function to calculate BLEURT
def calculate_bleurt(reference_texts, generated_texts):
    tokenizer = AutoTokenizer.from_pretrained("Elron/bleurt-base-128")
    model = AutoModelForSequenceClassification.from_pretrained("Elron/bleurt-base-128")
    model.eval()

    references = reference_texts
    candidates = generated_texts

    # 截断长文本，确保长度不超过模型的最大输入长度
    inputs = tokenizer(references, candidates, return_tensors='pt', truncation=True, padding=True, max_length=128)

    with torch.no_grad():
        # 计算BLEURT得分
        scores = model(**inputs)[0].squeeze()
    scores = scores.cpu().numpy()
    # print(np.mean(scores))  # 输出平均分
    return np.mean(scores)
    # scores = bleurt_scorer.score(references=reference_texts, candidates=generated_texts)
    # print(np.mean(scores))
    # return np.mean(scores)

# Define a function to calculate NUBIA (here we use a combination of BERTScore and ROUGE-L)
def calculate_nubia(reference_texts, generated_texts):
    # Here, NUBIA is a combination of BERTScore and ROUGE-L (for simplicity)
    bert_score_value = calculate_bertscore(reference_texts, generated_texts)
    rouge_score_value = calculate_rouge(reference_texts, generated_texts)
    # Weighted combination of BERTScore and ROUGE-L for NUBIA
    nubia_score = 0.7 * bert_score_value + 0.3 * rouge_score_value
    # print(nubia_score)
    return nubia_score

# Main function to compute all metrics for each pair of files
def compute_metrics_for_files(ground_truth_folder, generated_text_folder):
    ground_truth_files = os.listdir(ground_truth_folder)
    generated_files = os.listdir(generated_text_folder)

    # Initialize lists to store the metric values for all files
    bleu_values = []
    meteor_values = []
    rouge_l_values = []
    bertscore_values = []
    bleurt_values = []
    nubia_values = []

    # Iterate over files in the ground truth folder
    for gt_file in ground_truth_files:
        if gt_file.endswith('.txt') or gt_file.endswith('.json'):
            # Check if a corresponding generated file exists
            generated_file_txt = gt_file.split('.')[0] + '.txt'
            generated_file_json = gt_file.split('.')[0] + '.json'

            # Try to find the corresponding generated file, either in .txt or .json format
            if generated_file_txt in generated_files:
                generated_file = generated_file_txt
            elif generated_file_json in generated_files:
                generated_file = generated_file_json
            else:
                # If no corresponding generated file is found, skip this ground truth file
                continue
            # print(os.path.join(ground_truth_folder, gt_file))
            # print(os.path.join(generated_text_folder, generated_file))
            # Read the content from both files
            gt_text = read_file(os.path.join(ground_truth_folder, gt_file))
            generated_text = read_file(os.path.join(generated_text_folder, generated_file))
            # Calculate metrics for the current file pair
            reference_texts = [gt_text]
            generated_texts = [generated_text]
            
            bleu = calculate_bleu(reference_texts, generated_texts)
            meteor = calculate_meteor(reference_texts, generated_texts)
            rouge_l = calculate_rouge(reference_texts, generated_texts)
            bertscore = calculate_bertscore(reference_texts, generated_texts)
            bleurt = calculate_bleurt(reference_texts, generated_texts)
            nubia = calculate_nubia(reference_texts, generated_texts)
            
            # Append the metrics to their respective lists
            bleu_values.append(bleu)
            meteor_values.append(meteor)
            rouge_l_values.append(rouge_l)
            bertscore_values.append(bertscore)
            bleurt_values.append(bleurt)
            nubia_values.append(nubia)

    # Calculate the average values for each metric
    print(bleu_values)
    print(meteor_values)
    print(rouge_l_values)
    print(bertscore_values)
    print(bleurt_values)
    print(nubia_values)
    avg_bleu = np.mean(bleu_values) if bleu_values else 0.0
    avg_meteor = np.mean(meteor_values) if meteor_values else 0.0
    avg_rouge_l = np.mean(rouge_l_values) if rouge_l_values else 0.0
    avg_bertscore = np.mean(bertscore_values) if bertscore_values else 0.0
    avg_bleurt = np.mean([score for score in bleurt_values if score is not None]) if bleurt_values else 0.0
    # avg_bleurt = np.mean(bleurt_values) if bleurt_values else 0.0
    avg_nubia = np.mean(nubia_values) if nubia_values else 0.0

    # Print out the average values
    print("Average Metrics:")
    print(f"  Average BLEU-4: {avg_bleu:.4f}")
    print(f"  Average METEOR: {avg_meteor:.4f}")
    print(f"  Average ROUGE-L: {avg_rouge_l:.4f}")
    print(f"  Average BERTScore: {avg_bertscore:.4f}")
    print(f"  Average BLEURT: {avg_bleurt:.4f}")
    print(f"  Average NUBIA: {avg_nubia:.4f}")

    # Optionally, return these averages
    return {
        'Average BLEU-4': avg_bleu,
        'Average METEOR': avg_meteor,
        'Average ROUGE-L': avg_rouge_l,
        'Average BERTScore': avg_bertscore,
        'Average BLEURT': avg_bleurt,
        'Average NUBIA': avg_nubia
    }

# Example usage
ground_truth_folder = '/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/gpt_35_answer'
generated_text_folder = '/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/phi_3'

metrics_results = compute_metrics_for_files(ground_truth_folder, generated_text_folder)